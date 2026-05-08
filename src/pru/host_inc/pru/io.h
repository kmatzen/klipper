// Host stub for the pru-cgt <pru/io.h> header.
//
// The real header ships with pru-cgt and provides read_r31/write_r31
// (which read/write the PRU's R31 register - on PRU r31[31:30] holds
// pending host-interrupt status, and writes to r31 with bit 5 set
// inject INTC events) plus __delay_cycles() (cycle-accurate busy
// wait). On the host build (CONFIG_PRU_HOST_BUILD) we replace these
// with the host_pru.c emulation: read_r31 is composed from the host
// PRU INTC's pending bits, write_r31 dispatches a synthetic INTC
// event into _host_pru_intc, and __delay_cycles is a no-op since
// host time is wall-clock-driven (no PRU cycle to count).

#ifndef _HOST_PRU_IO_H
#define _HOST_PRU_IO_H

#include <stdint.h>

uint32_t host_pru_read_r31(void);
void host_pru_write_r31(uint32_t val);

static inline uint32_t read_r31(void) { return host_pru_read_r31(); }
static inline void write_r31(uint32_t val) { host_pru_write_r31(val); }

static inline void __delay_cycles(int n) { (void)n; }

#endif // _HOST_PRU_IO_H
