// Host emulation glue for src/pru/main.c (PRU host build).
//
// Compiled in only when CONFIG_PRU_HOST_BUILD=y. Provides:
//   - storage backing the host substitutes for SHARED_MEM, the IEP
//     timer, the PRU INTC, and the four AM335x GPIO MMIO blocks
//     (declared as host_pru.h-/internal.h-/pru_iep.h-extern symbols
//     from those headers).
//   - host_pru_init: parses argv (-I <slave-link>), opens a pty,
//     publishes the slave path as <slave-link> (mirroring what
//     src/linux/console.c does so scripts/test_klippy.py can treat
//     linuxpru.dict as a linuxprocess-style backend), spawns the
//     IEP timer thread + the pty I/O thread, and seeds
//     SHARED_MEM->signal so main.c's PRU0/PRU1 handshake completes
//     immediately.
//   - host_pru_read_r31 / host_pru_write_r31 / host_pru_irq_wait /
//     host_pru_intc_secr0_clear / host_pru_iep_cmp_sts_clear:
//     called from main.c (and from <pru/io.h> stub inlines) at
//     each pru-cgt-intrinsic site.
//   - IEP timer thread: continuously samples CLOCK_MONOTONIC into
//     CT_IEP.TMR_CNT (PRU runs at CONFIG_CLOCK_FREQ = 200 MHz), and
//     fires IEP_EVENT in PRU_INTC.SECR0 when CNT crosses CMP0 (one
//     fire per CMP0 value - tracked via last_fired_cmp0 to avoid
//     spurious re-fires while main.c is between
//     host_pru_iep_cmp_sts_clear and the new timer_set in
//     _irq_poll's IEP path).
//   - pty I/O thread: reads framed klipper command blocks from the
//     pty master, dispatches each command into the firmware via
//     SHARED_MEM->next_command (mirroring pru0.c::do_dispatch),
//     waits for the firmware's console_task to clear next_command
//     before pushing the next, and emits an ack frame to the pty
//     after each block.
//   - host_pru_send_response: called from main.c::console_sendf on
//     the firmware thread to encode + write a response frame
//     directly to the pty. We sidestep the SHARED_MEM->next_encoder
//     bridge (the firmware-to-pru0 path on real hardware) because
//     routing a va_list through a void* field is non-portable to
//     host x86_64 - va_list there is array-of-struct, not a plain
//     pointer like pru-cgt's. Since pru0 and the firmware share the
//     same process anyway on host, the SHARED_MEM bridge buys us
//     nothing.

#include "autoconf.h"

#ifdef CONFIG_PRU_HOST_BUILD

#define _GNU_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <pty.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include "board/io.h"     // readl / writel
#include "board/irq.h"    // irq_wait
#include "command.h"      // command_find_block, command_encode_and_frame,
                          // encode_acknak, MESSAGE_*
#include "host_pru.h"
#include "internal.h"     // SHARED_MEM, struct shared_mem, IEP_EVENT, ...
#include "pru/io.h"
#include "pru_iep.h"
#include "sched.h"

// ---- Storage for the symbols referenced via host_inc/ + internal.h ----

struct shared_mem _host_shared_mem;
struct host_iep _host_iep;
struct host_pru_intc _host_pru_intc;

// GPIO backing storage lives in src/pru/gpio.c (struct gpio_regs is
// file-static there); see the CONFIG_PRU_HOST_BUILD branch in that
// file.

// ADC backing storage. WANT_ADC is gated off for PRU (HAVE_GPIO_ADC
// is commented out in src/pru/Kconfig), so adc.c isn't built and
// nothing actually reads/writes &_host_adc - but ADC is referenced
// by the macro in internal.h, so the symbol has to exist or any
// (future) host build with WANT_ADC on would fail to link.
struct beaglebone_adc _host_adc;

// ---- Synchronization ----
//
// One global mutex (_host_lock) guards _host_iep, _host_pru_intc
// and the side-effects of the helper APIs below. _intc_cond is
// broadcast by the IEP thread + the I/O thread whenever a SECR0
// pending bit transitions 0 -> 1, so the firmware thread blocked
// in host_pru_irq_wait wakes promptly. _send_lock serialises pty
// writes between the I/O thread (ack frames) and the firmware
// thread (response frames) so the next_sequence counter inside
// command.c stays coherent.

static pthread_mutex_t _host_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t _intc_cond = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t _send_lock = PTHREAD_MUTEX_INITIALIZER;

static int _pty_master_fd = -1;
static volatile int _shutdown_requested = 0;

// Receive buffer for the I/O thread. Lives at file scope (not on
// the I/O thread's stack) so that command_decode_ptr / pointer-arg
// firmware handlers can use _host_receive_buf as the basis for
// pointer offsets back into the just-parsed command bytes - the
// generic command.c flow on 64-bit hosts encodes PT_buffer args as
// "offset from console_receive_buffer()" rather than raw pointers.
static uint8_t _host_receive_buf[MESSAGE_MAX];

void *
console_receive_buffer(void)
{
    return _host_receive_buf;
}

// Heap range for basecmd.c's alloc_chunk. PRU prod points the heap
// at the end of pru1 .data and caps it at the chip's 8KB data RAM
// (dynmem_end = 0x1F00 on PRU); for a host build that's senseless,
// so we substitute a host-process .bss region big enough to hold
// the firmware's per-config oid/move allocations. 128KB is well
// over the typical klippy printer config working set (move queue +
// per-stepper oids + thermal sensors, mid-tens of KB) without
// burning much of the host's RSS.
#define HOST_PRU_HEAP_SIZE (128 * 1024)
char _host_pru_heap[HOST_PRU_HEAP_SIZE];
void *const _host_pru_heap_end = _host_pru_heap + HOST_PRU_HEAP_SIZE;

// ---- IEP timer ----

static struct timespec _iep_epoch;
static int _iep_epoch_valid = 0;
static uint32_t _iep_last_fired_cmp0 = 0;
static int _iep_last_fired_cmp0_valid = 0;

static void
_update_tmr_cnt_locked(void)
{
    if (!_iep_epoch_valid) {
        clock_gettime(CLOCK_MONOTONIC, &_iep_epoch);
        _iep_epoch_valid = 1;
    }
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    uint64_t ns = (uint64_t)(now.tv_sec - _iep_epoch.tv_sec) * 1000000000ULL
                + (uint64_t)now.tv_nsec - (uint64_t)_iep_epoch.tv_nsec;
    // PRU IEP runs at CONFIG_CLOCK_FREQ = 200 MHz: 5 ns per tick.
    _host_iep.TMR_CNT = (uint32_t)(ns / 5);
}

static void *
_iep_timer_thread(void *arg)
{
    (void)arg;
    while (!_shutdown_requested) {
        pthread_mutex_lock(&_host_lock);
        _update_tmr_cnt_locked();
        uint32_t cnt = _host_iep.TMR_CNT;
        uint32_t cmp = _host_iep.TMR_CMP0;
        int armed = !(_host_iep.TMR_CMP_STS & 0x01);
        int already_fired = (_iep_last_fired_cmp0_valid
                             && _iep_last_fired_cmp0 == cmp);
        // CNT - CMP0 wraps every ~21 s (200 MHz, 32-bit). A signed
        // diff handles that correctly: diff>=0 means we're past CMP0
        // in the sense the firmware uses (timer_is_before).
        int32_t diff = (int32_t)(cnt - cmp);
        if (armed && !already_fired && diff >= 0
                && diff < (int32_t)0x40000000) {
            _host_iep.TMR_CMP_STS |= 0x01;
            _host_pru_intc.SECR0 |= (uint32_t)1 << IEP_EVENT;
            _iep_last_fired_cmp0 = cmp;
            _iep_last_fired_cmp0_valid = 1;
            pthread_cond_broadcast(&_intc_cond);
        }
        pthread_mutex_unlock(&_host_lock);
        // 50 us cadence: tighter than the firmware's typical
        // re-arming gap (~us), loose enough to keep the host CPU
        // spend negligible. The firmware's busy-wait in
        // timer_dispatch_many tolerates jitter of ~5 us at 200 MHz
        // (TIMER_MIN_TRY_TICKS = timer_from_us(2) -> 400 ticks).
        struct timespec ts = {.tv_sec = 0, .tv_nsec = 50 * 1000};
        nanosleep(&ts, NULL);
    }
    return NULL;
}

// ---- INTC + r31 helpers ----

uint32_t
host_pru_read_r31(void)
{
    pthread_mutex_lock(&_host_lock);
    uint32_t v = 0;
    if (_host_pru_intc.SECR0 & (((uint32_t)1 << IEP_EVENT)
                                | ((uint32_t)1 << KICK_PRU1_EVENT)))
        v |= (uint32_t)1 << (WAKE_PRU1_IRQ + R31_IRQ_OFFSET);
    pthread_mutex_unlock(&_host_lock);
    return v;
}

void
host_pru_write_r31(uint32_t val)
{
    // The firmware only triggers write_r31 to kick PRU0 from
    // console_sendf, but we route console_sendf around SHARED_MEM
    // (see host_pru_send_response below) on host build, so by the
    // time write_r31 is called there's nothing left to do. Other
    // INTC events (KICK_ARM_EVENT etc.) only matter for
    // rpmsg/virtio on real PRU and we don't model that.
    (void)val;
}

void
host_pru_intc_secr0_clear(uint32_t bits)
{
    pthread_mutex_lock(&_host_lock);
    _host_pru_intc.SECR0 &= ~bits;
    // No broadcast: clearing pending bits never wakes a waiter
    // (irq_wait only sleeps when no bits are set).
    pthread_mutex_unlock(&_host_lock);
}

void
host_pru_iep_cmp_sts_clear(uint32_t bits)
{
    pthread_mutex_lock(&_host_lock);
    _host_iep.TMR_CMP_STS &= ~bits;
    pthread_mutex_unlock(&_host_lock);
}

void
host_pru_irq_wait(void)
{
    pthread_mutex_lock(&_host_lock);
    if (!(_host_pru_intc.SECR0 & (((uint32_t)1 << IEP_EVENT)
                                  | ((uint32_t)1 << KICK_PRU1_EVENT)))) {
        // Cap at 1 ms so the firmware thread always makes forward
        // progress even if a signal goes missing. The IEP thread
        // broadcasts on every fire so the common case wakes within
        // its 50us polling interval.
        struct timespec ts;
        clock_gettime(CLOCK_REALTIME, &ts);
        ts.tv_nsec += 1000 * 1000;
        if (ts.tv_nsec >= 1000000000L) {
            ts.tv_sec += 1;
            ts.tv_nsec -= 1000000000L;
        }
        pthread_cond_timedwait(&_intc_cond, &_host_lock, &ts);
    }
    pthread_mutex_unlock(&_host_lock);
}

// ---- Response send (firmware thread context) ----

static void
_write_pty_locked_unlocked(const uint8_t *buf, size_t msglen)
{
    if (_pty_master_fd < 0)
        return;
    size_t off = 0;
    while (off < msglen) {
        ssize_t n = write(_pty_master_fd, buf + off, msglen - off);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                // Reader fell behind; drop the rest. The host will
                // retransmit the prompting command after its own
                // timeout - matching the lossy behavior of a real
                // serial link.
                break;
            fprintf(stderr,
                    "host_pru: pty write error: %s\n", strerror(errno));
            break;
        }
        off += (size_t)n;
    }
}

void
host_pru_send_response(const struct command_encoder *ce, va_list args)
{
    // Called from main.c::console_sendf on the firmware thread.
    // The va_list lives on the caller's stack (command_sendf
    // started it via va_start), so it's safe to consume here. We
    // bypass SHARED_MEM->next_encoder because routing the va_list
    // through a void* field is non-portable to host x86_64
    // (va_list is array-of-struct, not just a pointer).
    uint8_t buf[MESSAGE_MAX];
    pthread_mutex_lock(&_send_lock);
    uint_fast8_t msglen = command_encode_and_frame(buf, sizeof(buf), ce, args);
    _write_pty_locked_unlocked(buf, msglen);
    pthread_mutex_unlock(&_send_lock);
}

// ---- pty I/O thread ----

extern const struct command_encoder encode_acknak;

// Varargs trampoline so we can hand command_encode_and_frame a
// va_list created from va_start (memset-zeroing a va_list and
// passing it is undefined on every host ABI we care about). The
// vararg has no extra args; encode_acknak has num_params=0 so
// command_encodef never reaches va_arg.
static void
_send_frame_via_varargs(const struct command_encoder *ce, ...)
{
    uint8_t buf[MESSAGE_MAX];
    va_list args;
    va_start(args, ce);
    pthread_mutex_lock(&_send_lock);
    uint_fast8_t msglen = command_encode_and_frame(buf, sizeof(buf), ce, args);
    _write_pty_locked_unlocked(buf, msglen);
    pthread_mutex_unlock(&_send_lock);
    va_end(args);
}

static void
_send_ack_frame(void)
{
    // Don't go through command_send_ack -> command_sendf ->
    // console_sendf - that's the firmware's send path and the I/O
    // thread calling it would reenter host_pru_send_response on the
    // wrong stack. Build the ack frame ourselves and write it
    // directly under _send_lock so the next_sequence counter inside
    // command.c stays linearised against firmware-thread sends.
    _send_frame_via_varargs(&encode_acknak);
}

static void
_dispatch_command(uint8_t *buf, uint_fast8_t pop_count)
{
    // Mirror pru0.c::do_dispatch: walk every command in the block,
    // populate SHARED_MEM->next_command_args + next_command, signal
    // KICK_PRU1_EVENT, wait for the firmware's console_task to
    // clear next_command, then move to the next command.
    uint8_t *p = &buf[MESSAGE_HEADER_SIZE];
    uint8_t *msgend = &buf[pop_count - MESSAGE_TRAILER_SIZE];
    while (p < msgend) {
        uint_fast16_t cmdid = command_parse_msgid(&p);
        const struct command_parser *index = SHARED_MEM->command_index;
        uint32_t index_size = SHARED_MEM->command_index_size;
        if (!cmdid || cmdid >= index_size) {
            // Unknown command id - mirror pru0's send_pru1_shutdown
            // by injecting the shutdown_handler.
            const struct command_parser *cp = SHARED_MEM->shutdown_handler;
            if (!cp)
                return;
            while (!_shutdown_requested) {
                const struct command_parser *cur =
                    __atomic_load_n(&SHARED_MEM->next_command,
                                    __ATOMIC_ACQUIRE);
                if (!cur)
                    break;
                struct timespec ts = {.tv_sec = 0, .tv_nsec = 100 * 1000};
                nanosleep(&ts, NULL);
            }
            __atomic_store_n(&SHARED_MEM->next_command, cp,
                             __ATOMIC_RELEASE);
            pthread_mutex_lock(&_host_lock);
            _host_pru_intc.SECR0 |= (uint32_t)1 << KICK_PRU1_EVENT;
            pthread_cond_broadcast(&_intc_cond);
            pthread_mutex_unlock(&_host_lock);
            return;
        }
        const struct command_parser *cp = &index[cmdid];
        p = command_parsef(p, msgend, cp,
                           (uint32_t *)SHARED_MEM->next_command_args);

        // Release-store next_command so the firmware thread sees
        // both next_command_args (filled by command_parsef above)
        // and the new parser pointer in one consistent transition.
        // The matching acquire-load is the one console_task does
        // implicitly via the mutex-less polling read it inherits
        // from the original PRU code (which on host needs the
        // memory ordering to be explicit).
        __atomic_store_n(&SHARED_MEM->next_command, cp, __ATOMIC_RELEASE);
        pthread_mutex_lock(&_host_lock);
        _host_pru_intc.SECR0 |= (uint32_t)1 << KICK_PRU1_EVENT;
        pthread_cond_broadcast(&_intc_cond);
        pthread_mutex_unlock(&_host_lock);

        // Wait for console_task to consume + clear next_command
        // before queuing the next. Acquire-load pairs with the
        // release-store in main.c::console_task on host build so
        // we observe the cleared field reliably under aarch64's
        // weak memory ordering.
        while (!_shutdown_requested) {
            const struct command_parser *cur =
                __atomic_load_n(&SHARED_MEM->next_command, __ATOMIC_ACQUIRE);
            if (!cur)
                break;
            struct timespec ts = {.tv_sec = 0, .tv_nsec = 100 * 1000};
            nanosleep(&ts, NULL);
        }
    }
}

static void *
_pty_io_thread(void *arg)
{
    (void)arg;
    // Wait until the firmware finishes its boot handshake -
    // command_index isn't valid for dispatch until then.
    while (!_shutdown_requested
           && readl(&SHARED_MEM->signal) != SIGNAL_PRU1_READY) {
        struct timespec ts = {.tv_sec = 0, .tv_nsec = 1 * 1000 * 1000};
        nanosleep(&ts, NULL);
    }

    int buf_len = 0;
    struct pollfd pfd = {.fd = _pty_master_fd, .events = POLLIN};
    while (!_shutdown_requested) {
        int rc = poll(&pfd, 1, 50);
        if (rc < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (rc == 0)
            continue;
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            // Other end closed; klippy-side disconnect. Idle until
            // the parent process tears us down.
            struct timespec ts = {.tv_sec = 0, .tv_nsec = 50 * 1000 * 1000};
            nanosleep(&ts, NULL);
            continue;
        }
        ssize_t n = read(_pty_master_fd, _host_receive_buf + buf_len,
                         (ssize_t)sizeof(_host_receive_buf) - buf_len);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR)
                continue;
            break;
        }
        if (n == 0)
            continue;
        buf_len += (int)n;

        // Drain as many complete blocks as the buffer holds.
        while (buf_len > 0) {
            uint_fast8_t pop_count = 0;
            int_fast8_t ret = command_find_block(_host_receive_buf,
                                                 buf_len, &pop_count);
            if (!ret)
                break;
            if (ret > 0) {
                _dispatch_command(_host_receive_buf, pop_count);
                _send_ack_frame();
            }
            // ret < 0 (bad sync) and ret > 0 (good block) both pop.
            if (pop_count >= (uint_fast8_t)buf_len) {
                buf_len = 0;
            } else {
                memmove(_host_receive_buf,
                        _host_receive_buf + pop_count,
                        buf_len - pop_count);
                buf_len -= pop_count;
            }
        }
    }
    return NULL;
}

// ---- Initialisation ----

static int
_set_non_blocking(int fd)
{
    int flags = fcntl(fd, F_GETFL);
    if (flags < 0)
        return -1;
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK);
}

static int
_open_pty_and_publish(const char *slave_link)
{
    struct termios ti;
    memset(&ti, 0, sizeof(ti));
    int mfd, sfd;
    int rc = openpty(&mfd, &sfd, NULL, &ti, NULL);
    if (rc < 0) {
        fprintf(stderr, "host_pru: openpty: %s\n", strerror(errno));
        return -1;
    }
    if (_set_non_blocking(mfd) < 0) {
        fprintf(stderr, "host_pru: set_nonblocking: %s\n", strerror(errno));
        return -1;
    }
    fcntl(mfd, F_SETFD, FD_CLOEXEC);
    fcntl(sfd, F_SETFD, FD_CLOEXEC);

    char *tname = ttyname(sfd);
    if (!tname) {
        fprintf(stderr, "host_pru: ttyname: %s\n", strerror(errno));
        return -1;
    }
    unlink(slave_link);
    if (symlink(tname, slave_link) < 0) {
        fprintf(stderr, "host_pru: symlink %s: %s\n",
                slave_link, strerror(errno));
        return -1;
    }
    chmod(tname, 0660);

    _pty_master_fd = mfd;
    return 0;
}

void
host_pru_init(int argc, char **argv)
{
    // SIGPIPE on pty write while klippy is closed would terminate
    // us before we can recover; handle EPIPE return values directly
    // in _write_pty_locked_unlocked instead.
    signal(SIGPIPE, SIG_IGN);

    const char *slave_link = NULL;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "-I") == 0 && i + 1 < argc) {
            slave_link = argv[++i];
        }
    }
    if (!slave_link) {
        fprintf(stderr, "host_pru: missing -I <slave-link>\n");
        exit(2);
    }

    memset(&_host_shared_mem, 0, sizeof(_host_shared_mem));
    memset(&_host_iep, 0, sizeof(_host_iep));
    memset(&_host_pru_intc, 0, sizeof(_host_pru_intc));

    if (_open_pty_and_publish(slave_link) < 0)
        exit(3);

    // Seed the PRU0 -> PRU1 handshake. main.c spins on this.
    writel(&_host_shared_mem.signal, SIGNAL_PRU0_WAITING);

    pthread_t iep_tid, io_tid;
    if (pthread_create(&iep_tid, NULL, _iep_timer_thread, NULL) != 0) {
        fprintf(stderr, "host_pru: pthread_create iep failed\n");
        exit(4);
    }
    pthread_detach(iep_tid);
    if (pthread_create(&io_tid, NULL, _pty_io_thread, NULL) != 0) {
        fprintf(stderr, "host_pru: pthread_create io failed\n");
        exit(4);
    }
    pthread_detach(io_tid);
}

#endif // CONFIG_PRU_HOST_BUILD
