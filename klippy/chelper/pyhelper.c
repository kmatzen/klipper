// Helper functions for C / Python interface
//
// Copyright (C) 2016-2018  Kevin O'Connor <kevin@koconnor.net>
//
// This file may be distributed under the terms of the GNU GPLv3 license.

#include <errno.h> // errno
#include <fcntl.h> // open
#include <stdarg.h> // va_start
#include <stdint.h> // uint8_t
#include <stdio.h> // fprintf
#include <stdlib.h> // getenv
#include <string.h> // strerror
#include <sys/mman.h> // mmap
#include <sys/stat.h> // fstat
#include <time.h> // struct timespec
#include <unistd.h> // close
#include <sys/prctl.h>  // prctl
#include "compiler.h" // __visible
#include "pyhelper.h" // get_monotonic

// Optional sim-time mode: if KLIPPY_SIM_TIME_FILE is set in the
// environment at process start, get_monotonic() reads its time
// from a memory-mapped double the simavr bridge updates each tick
// instead of clock_gettime(). This decouples klippy's view of
// "time" from wall-clock so deterministic simulation tests run
// correctly regardless of host CPU load.
static volatile double *sim_time_ptr = NULL;
static int sim_time_initialized = 0;

static void
sim_time_init(void)
{
    sim_time_initialized = 1;
    const char *path = getenv("KLIPPY_SIM_TIME_FILE");
    if (!path || !*path)
        return;
    int fd = open(path, O_RDONLY);
    if (fd < 0)
        return;
    void *p = mmap(NULL, sizeof(double), PROT_READ, MAP_SHARED, fd, 0);
    close(fd);
    if (p == MAP_FAILED)
        return;
    sim_time_ptr = (volatile double *)p;
}

// Return the monotonic system time as a double
double __visible
get_monotonic(void)
{
    if (!sim_time_initialized)
        sim_time_init();
    if (sim_time_ptr)
        return *sim_time_ptr;
    struct timespec ts;
    int ret = clock_gettime(CLOCK_MONOTONIC_RAW, &ts);
    if (ret) {
        report_errno("clock_gettime", ret);
        return 0.;
    }
    return (double)ts.tv_sec + (double)ts.tv_nsec * .000000001;
}

// Fill a 'struct timespec' with a system time stored in a double
struct timespec
fill_time(double time)
{
    time_t t = time;
    return (struct timespec) {t, (time - t)*1000000000. };
}

static void
default_logger(const char *msg)
{
    fprintf(stderr, "%s\n", msg);
}

static void (*python_logging_callback)(const char *msg) = default_logger;

void __visible
set_python_logging_callback(void (*func)(const char *))
{
    python_logging_callback = func;
}

// Log an error message
void
errorf(const char *fmt, ...)
{
    char buf[512];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    buf[sizeof(buf)-1] = '\0';
    python_logging_callback(buf);
}

// Report 'errno' in a message written to stderr
void
report_errno(char *where, int rc)
{
    int e = errno;
    errorf("Got error %d in %s: (%d)%s", rc, where, e, strerror(e));
}

// Return a hex character for a given number
#define GETHEX(x) ((x) < 10 ? '0' + (x) : 'a' + (x) - 10)

// Translate a binary string into an ASCII string with escape sequences
char *
dump_string(char *outbuf, int outbuf_size, char *inbuf, int inbuf_size)
{
    char *outend = &outbuf[outbuf_size-5], *o = outbuf;
    uint8_t *inend = (void*)&inbuf[inbuf_size], *p = (void*)inbuf;
    while (p < inend && o < outend) {
        uint8_t c = *p++;
        if (c > 31 && c < 127 && c != '\\') {
            *o++ = c;
            continue;
        }
        *o++ = '\\';
        *o++ = 'x';
        *o++ = GETHEX(c >> 4);
        *o++ = GETHEX(c & 0x0f);
    }
    *o = '\0';
    return outbuf;
}

// Set custom thread names
int __visible
set_thread_name(char name[16])
{
    return prctl(PR_SET_NAME, name);
}
