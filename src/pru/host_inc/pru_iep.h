// Host stub for the pru-cgt <pru_iep.h> header.
//
// The real header (and the related pru-cgt CT_* macros) provides
// memory-mapped peripheral overlays for the PRU subsystem - chiefly
// CT_IEP for the IEP (Industrial Ethernet Peripheral) timer that
// drives klipper's PRU timer and PRU_INTC for inter-PRU / host
// kicks. Both vanish in a host build; we replace them with plain
// host structs (storage in src/pru/host_pru.c). A few register
// fields require write-1-to-clear semantics that simple struct
// stores can't capture - those sites in src/pru/main.c branch on
// CONFIG_PRU_HOST_BUILD to call the host_pru_*_clear helpers
// declared in src/pru/host_pru.h instead of writing directly.

#ifndef _HOST_PRU_IEP_H
#define _HOST_PRU_IEP_H

#include <stdint.h>

// IEP timer surface, narrow to the fields klipper's pru main.c
// touches (TMR_GLB_CFG / TMR_CMP_CFG / TMR_CNT / TMR_CMP0 /
// TMR_CMP_STS). Field names match pru-cgt so the firmware source
// reads identically on both targets.
struct host_iep {
    volatile uint32_t TMR_GLB_CFG;
    volatile uint32_t TMR_CMP_CFG;
    volatile uint32_t TMR_CNT;
    volatile uint32_t TMR_CMP0;
    volatile uint32_t TMR_CMP_STS;
};

// PRU INTC surface. SECR0 is the only field klipper's pru main.c
// reads/writes from PRU1; SECR1 is here because pru0.c clears it on
// boot (we skip pru0.c on host but keep the field for binary-layout
// parity with the real INTC).
struct host_pru_intc {
    volatile uint32_t SECR0;
    volatile uint32_t SECR1;
};

extern struct host_iep _host_iep;
extern struct host_pru_intc _host_pru_intc;

#define CT_IEP _host_iep
#define PRU_INTC _host_pru_intc

#endif // _HOST_PRU_IEP_H
