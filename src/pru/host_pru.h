// Host emulation entry points for src/pru/main.c (PRU host build).
//
// This header is force-included into every translation unit by
// src/pru/Makefile (`-include src/pru/host_pru.h`) when
// CONFIG_PRU_HOST_BUILD is set, so the PRU sources compile against
// host gcc as if pru-cgt's intrinsics, the IEP timer, and the PRU
// INTC were available locally. A few register fields require
// write-1-to-clear semantics that plain struct stores can't capture
// - main.c branches on CONFIG_PRU_HOST_BUILD to call the
// host_pru_*_clear helpers declared below at the four affected
// sites (timer_kick + _irq_poll).

#ifndef _HOST_PRU_H
#define _HOST_PRU_H

// Pull in the build-time CONFIG_PRU_HOST_BUILD define before any
// other src/ header gets a chance to test it. host_pru.h is
// force-included via -include only on the host build path, so
// reading autoconf.h here is the simplest way to make the
// CONFIG_PRU_HOST_BUILD #ifdef branches in internal.h, gpio.c, and
// main.c match what the rest of this header decides.
#include "autoconf.h"

#ifdef CONFIG_PRU_HOST_BUILD

#include <stdarg.h>
#include <stdint.h>

struct command_encoder;

// Encode + write a klipper response frame straight to the pty
// master, bypassing the SHARED_MEM->next_encoder bridge (the PRU
// firmware's normal pru1->pru0 path). Called from main.c's
// console_sendf when CONFIG_PRU_HOST_BUILD is set; the firmware
// thread is the caller, so the va_list is alive on the same stack
// for the duration of this call.
void host_pru_send_response(const struct command_encoder *ce, va_list args);

// Set up the host SHARED_MEM, IEP timer thread, and pty-backed
// console bridge. Called from main.c's main() before the PRU0/PRU1
// signal handshake so that the firmware's spin-wait on
// SHARED_MEM->signal == SIGNAL_PRU0_WAITING completes immediately
// (host_pru_init seeds the signal field).
void host_pru_init(int argc, char **argv);

// Write-1-to-clear emulation for the PRU INTC SECR0 and IEP
// CMP_STS registers. The firmware writes a bitmask to these and
// expects the named bits to be cleared while other bits stay
// pending. A plain `_host_pru_intc.SECR0 = bits` would overwrite
// the whole register; these helpers atomic-AND-NOT and broadcast
// the host_pru wait condvar so the IEP / I/O threads notice the
// state change.
void host_pru_intc_secr0_clear(uint32_t bits);
void host_pru_iep_cmp_sts_clear(uint32_t bits);

// Suspend the firmware thread until either the IEP timer fires
// (TMR_CNT crosses TMR_CMP0) or the I/O thread sets KICK_PRU1_EVENT
// in PRU INTC SECR0. Replaces the `slp 1` instruction in main.c's
// irq_wait().
void host_pru_irq_wait(void);

#endif // CONFIG_PRU_HOST_BUILD

#endif // _HOST_PRU_H
