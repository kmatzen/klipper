# SAME70 XDMAC channel-0 busy-wait synthesis stub.
#
# klipper SAME70 firmware (src/atsam/same70_sysinit.c) uses XDMAC
# channel 0 to copy the .text section from flash (0x400000, the LMA
# given by CONFIG_ARMCM_ITCM_FLASH_MIRROR_START) to ITCM (0x0, the
# .text VMA = CONFIG_FLASH_APPLICATION_ADDRESS) at boot:
#
#     enable_pclock(ID_XDMAC);
#     REG_XDMAC_CSA0 = CONFIG_ARMCM_ITCM_FLASH_MIRROR_START;
#     REG_XDMAC_CDA0 = CONFIG_FLASH_APPLICATION_ADDRESS;
#     REG_XDMAC_CUBC0 = ...;
#     XDMAC->XDMAC_GE = XDMAC_GE_EN0;
#     while (XDMAC->XDMAC_GS & XDMAC_GS_ST0) ;       // wait DMA done
#     while (!(REG_XDMAC_CIS0 & XDMAC_CIS_BIS)) ;    // wait BIS latch
#
# Renode's upstream sam_e70.repl tags XDMAC from the ATSAME70Q21 SVD
# as a generic stub - reads return 0. The first loop exits fine
# (XDMAC_GS_ST0 is bit 0; 0 & 1 == 0), but the second spins forever
# because CIS0 also returns 0 and BIS never latches.
#
# In our test harness we side-step the DMA copy entirely: the runner
# loads the ELF with `useVirtualAddress=true` so .text bytes land at
# the VMA (0x0) directly, the same place the DMA would put them.
# This stub then just synthesises CIS0.BIS = 1 so the firmware sees
# the (no-op) copy as already done and progresses to ITCM enable.
#
# Stub region covers channel-0 registers at XDMAC base + 0x50..0x90
# (CIE0/CID0/CIM0/CIS0/CSA0/CDA0/CNDA0/CNDC0/CUBC0/CBC0/CC0/...). All
# writes are dict-backed storeback. CIS0 (offset 0x0C from stub base)
# is the only synthesised read; everything else falls back to the
# stored value (or 0 default).

regs = {}

# CIS_BIS (End of Block Interrupt Status). klipper reads CIS0 only
# to gate on this bit; no other CIS bits matter for the firmware
# path covered here.
CIS_BIS = 1 << 0

# Stub base = XDMAC_BASE + 0x50, so offset 0x0C maps to the absolute
# REG_XDMAC_CIS0 = 0x4007805C.
CIS0_OFFSET = 0x0C

if request.IsInit:
    pass
elif request.IsRead:
    if request.Offset == CIS0_OFFSET:
        request.Value = CIS_BIS
    else:
        request.Value = regs.get(request.Offset, 0)
elif request.IsWrite:
    regs[request.Offset] = request.Value
