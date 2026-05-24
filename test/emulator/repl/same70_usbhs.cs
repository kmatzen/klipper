//
// SAME70 USBHS (a.k.a. UOTGHS) device-mode model for Renode.
//
// Renode upstream has no SAM USB-OTG / USBHS peripheral (none of the
// existing models in renode-infrastructure/src/Emulator/Peripherals/
// Peripherals/USB - Cadence_USB, MPFS_USB, NRF_USBD, ValentyUSB - have
// the Atmel register layout). klipper SAME70 firmware compiled with
// CONFIG_ATSAM_USB drives the controller through src/atsam/sam3_usb.c
// (shared with SAM3X via the same70_usb.h alias header) and expects:
//
//   * Control register file at 0x40038000:
//       0x000 DEVCTRL    Device general control - SPDCONF/ADDEN/UADD
//       0x004 DEVISR     Global interrupt status (EORST + per-EP PEP_n)
//       0x008 DEVICR     W1C of DEVISR latched bits
//       0x010 DEVIMR     Internal interrupt mask (read-only mirror of
//                        the IER/IDR set/clear pair)
//       0x014 DEVIDR     W1C clear of DEVIMR
//       0x018 DEVIER     W1S set of DEVIMR
//       0x01C DEVEPT     EP enable bits (firmware writes EPENn flags)
//       0x100..0x12C DEVEPTCFG[0..11]   Per-EP configure (ALLOC bit
//                                        commits the FIFO bank)
//       0x130..0x15C DEVEPTISR[0..11]   Per-EP status (TXINI/RXOUTI/
//                                        RXSTPI in low 3 bits + BYCT
//                                        at bit 20)
//       0x160..0x18C DEVEPTICR[0..11]   W1C of low DEVEPTISR latches
//       0x1C0..0x1EC DEVEPTIMR[0..11]   Per-EP IRQ mask mirror
//       0x1F0..0x21C DEVEPTIER[0..11]   W1S set of DEVEPTIMR
//       0x220..0x24C DEVEPTIDR[0..11]   W1C clear of DEVEPTIMR (also
//                                        carries FIFOCONC at bit 14,
//                                        the bank-release signal)
//       0x800 USBHS_CTRL Top-level enable (USBE/FRZCLK/UIMOD)
//
//   * Endpoint FIFO DPRAM at 0xa0100000 - byte-addressable. klipper
//     accesses each EP's FIFO via `usb_fifo(ep) = 0xa0100000 + ep*0x8000`
//     and walks bytes sequentially: each write advances the controller's
//     internal byte-count, which is reported as DEVEPTISR.BYCT for
//     OUT-direction packets and consumed by the host on the next IN
//     transfer when the firmware clears TXINI.
//
// Both regions land on the SAME peripheral instance via the .repl's
// BusMultiRegistration syntax: the default `sysbus 0x40038000` covers
// the control region and `sysbus new Bus.BusMultiRegistration {
// address: 0xa0100000; size: 0x80000; region: "fifo" }` adds the
// DPRAM. The fifo accessors are tagged with [ConnectionRegion("fifo")]
// so Renode's bus dispatch routes byte-level fifo accesses to the
// dedicated methods rather than the default ReadDoubleWord/WriteDoubleWord.
//
// To avoid simulating a full USB host (descriptor exchange, address
// negotiation, configuration walk - see generic/usb_cdc.c's
// usb_state_ready dispatch) the model uses a "skip-enum cheat": after
// the firmware enables EORST in DEVIER, the model fires EORST once,
// then on the firmware's first read of DEVEPTISR[0] with RXSTPES
// enabled it injects a single SETUP packet of {bRequestType=0,
// bRequest=USB_REQ_SET_CONFIGURATION (=9), wValue=1, wIndex=0,
// wLength=0}. klipper firmware's usb_req_set_configuration calls
// usb_set_configure() (which DEVEPTCFGs the bulk endpoints) and acks
// with usb_send_ep0(NULL, 0). At that point bulk EP1 (BULK_IN) and
// EP2 (BULK_OUT) are configured and we treat them as a UART pipe to
// the host pty - exactly the path UartPtyTerminal already takes via
// UARTBase.TransmitCharacter / UARTBase.WriteChar.
//
// The skip-enum cheat is invisible to klippy: after enumeration is
// done the firmware behaves identically to a real USB-CDC build (same
// Klipper protocol bytes over BULK_IN/BULK_OUT), so klippy's identify
// + clocksync handshake works against the pty without any
// modification.
//
// IRQ output goes to USBHS_IRQn = 34 on SAME70 - wired via the .repl's
// `IRQ -> nvic@34` line.
//
// Loaded into the Renode runtime via `i @<path>/same70_usbhs.cs`
// emitted by renode_launcher._render_resc before LoadPlatformDescription.

using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.UART;

namespace Antmicro.Renode.Peripherals.USB
{
    public class SAM_USBHS : UARTBase, IDoubleWordPeripheral, IKnownSize
    {
        public SAM_USBHS(IMachine machine) : base(machine)
        {
            IRQ = new GPIO();
            for(int i = 0; i < EndpointCount; i++)
            {
                endpoints[i] = new EndpointState();
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            switch(offset)
            {
            case (long)Registers.DEVCTRL:    return devCtrl;
            case (long)Registers.DEVISR:     return ReadDevIsr();
            case (long)Registers.DEVIMR:     return devImr;
            case (long)Registers.DEVEPT:     return devEpt;
            case (long)Registers.DEVFNUM:    return 0;
            case (long)Registers.USBHS_CTRL: return usbhsCtrl;
            }
            if(IsEpRegister(offset, EPTCFG_BASE, out int idx))
                return endpoints[idx].Cfg;
            if(IsEpRegister(offset, EPTISR_BASE, out idx))
                return ReadEptIsr(idx);
            if(IsEpRegister(offset, EPTIMR_BASE, out idx))
                return endpoints[idx].Imr;
            this.Log(LogLevel.Warning,
                "SAM_USBHS: read from unmapped offset 0x{0:X}", offset);
            return 0;
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch(offset)
            {
            case (long)Registers.DEVCTRL:
                devCtrl = value;
                return;
            case (long)Registers.DEVICR:
                // W1C of EORST / SUSP / WAKEUP latches. PEP_n bits live
                // in DEVISR but are derived from per-EP state and not
                // clearable via DEVICR (firmware uses DEVEPTICR for
                // those).
                devIsrLatched &= ~value;
                UpdateIRQ();
                return;
            case (long)Registers.DEVIDR:
                devImr &= ~value;
                UpdateIRQ();
                return;
            case (long)Registers.DEVIER:
            {
                uint newlyEnabled = value & ~devImr;
                devImr |= value;
                // klipper's usbserial_init writes DEVIER = EORSTES as
                // its last act before returning to the scheduler -
                // armcm_enable_irq for UOTGHS_IRQn ran earlier in the
                // same function so the NVIC line is already unmasked.
                // Latch EORST in DEVISR right away to drive the IRQ;
                // the firmware's UOTGHS_Handler reads DEVISR, sees
                // EORST, runs handle_end_reset (which configures EP0
                // and writes DEVICR_EORSTC), and returns - we don't
                // need to wait for any other event before asserting.
                if((newlyEnabled & EORST_BIT) != 0
                    && !endOfResetFired)
                {
                    devIsrLatched |= EORST_BIT;
                    endOfResetFired = true;
                }
                UpdateIRQ();
                return;
            }
            case (long)Registers.DEVEPT:
                // Plain storeback - the EP enable bits are written by
                // handle_end_reset and usb_set_configure but never read
                // back by the firmware. The "is this EP active" flag
                // we actually care about is the ALLOC bit in DEVEPTCFG.
                devEpt = value;
                return;
            case (long)Registers.USBHS_CTRL:
                usbhsCtrl = value;
                return;
            }
            if(IsEpRegister(offset, EPTCFG_BASE, out int idx))
            {
                endpoints[idx].Cfg = value;
                if((value & EPTCFG_ALLOC) != 0)
                {
                    // Real silicon: ALLOC commits the FIFO bank to the
                    // EP and resets the byte counter. Initial TXINI=1
                    // (FIFO empty, ready to accept an IN packet from
                    // the firmware) per datasheet 39.6.2.5.
                    endpoints[idx].Allocated = true;
                    endpoints[idx].Isr = TXINI_BIT;
                    endpoints[idx].TxByteCount = 0;
                    endpoints[idx].RxByteCount = 0;
                    // klippy may have already queued bytes via the pty
                    // before the firmware finished USB enumeration -
                    // those landed in the UART rx queue and were left
                    // there because BULK_OUT was not allocated yet.
                    // Flush the queue now that the EP exists so the
                    // first OUT packet is the leading slice of klippy's
                    // identify request, not a NAK'd black hole.
                    if(idx == BULK_OUT)
                    {
                        CharWritten();
                    }
                }
                return;
            }
            if(IsEpRegister(offset, EPTICR_BASE, out idx))
            {
                ClearEptStatus(idx, value);
                return;
            }
            if(IsEpRegister(offset, EPTIER_BASE, out idx))
            {
                endpoints[idx].Imr |= value;
                // Inject the synthetic SET_CONFIGURATION SETUP as soon as
                // the firmware arms RXSTPES on the (already allocated) EP0
                // - handle_end_reset() writes DEVEPTCFG[0]|ALLOC then
                // DEVEPTIER[0]=RXSTPES back to back. Doing it here (rather
                // than waiting for the firmware to read DEVEPTISR[0]) makes
                // enumeration self-start: otherwise the firmware waits for
                // the RXSTPI IRQ that the injection itself raises, a
                // chicken-and-egg that only resolved when an unrelated
                // (SysTick-rate-dependent) DEVEPTISR[0] poll happened to
                // fire. With this, enumeration no longer depends on the
                // SysTick rate, so the platform can use the correct 300MHz
                // SysTick (= DWT = CONFIG_CLOCK_FREQ) that avoids the
                // reason-56 "Rescheduled timer in the past" shutdown.
                if(idx == 0 && (value & RXSTPI_BIT) != 0
                    && endpoints[0].Allocated && !setupInjected)
                    StageSetupPacket();
                UpdateIRQ();
                return;
            }
            if(IsEpRegister(offset, EPTIDR_BASE, out idx))
            {
                // Bit 14 (FIFOCONC) is the FIFO control bit firmware
                // pulses after each packet to release the bank back to
                // the controller. The model has only single-bank
                // semantics (sufficient for klipper's request/response
                // cadence), so FIFOCONC on OUT means "buffer free
                // again, refill from any queued bytes".
                if((value & EPT_FIFOCONC) != 0)
                {
                    OnFifoCon(idx);
                }
                endpoints[idx].Imr &= ~value;
                UpdateIRQ();
                return;
            }
            // DEVEPTIFR (write-1-set status) is unused by klipper.
            if(offset >= EPTIFR_BASE && offset < EPTIFR_BASE + 0x30)
                return;
            this.Log(LogLevel.Warning,
                "SAM_USBHS: write 0x{0:X} to unmapped offset 0x{1:X}",
                value, offset);
        }

        // FIFO byte access through BusMultiRegistration("fifo"):
        // klipper reads/writes endpoint banks one byte at a time via
        // the linear DPRAM at 0xa0100000+ep*0x8000. Each write advances
        // the controller's internal TX byte count; each read leaves
        // the buffered packet in place (firmware's usb_read_packet
        // scans byteOffset 0..len-1 sequentially).
        [ConnectionRegion("fifo")]
        public byte ReadByteFromFifo(long offset)
        {
            int ep = (int)(offset / EpStride);
            int byteOffset = (int)(offset % EpStride);
            if(ep < 0 || ep >= EndpointCount)
                return 0;
            byte[] buf = endpoints[ep].RxBuffer;
            if(buf == null || byteOffset < 0 || byteOffset >= buf.Length)
                return 0;
            return buf[byteOffset];
        }

        [ConnectionRegion("fifo")]
        public void WriteByteToFifo(long offset, byte value)
        {
            int ep = (int)(offset / EpStride);
            int byteOffset = (int)(offset % EpStride);
            if(ep < 0 || ep >= EndpointCount)
                return;
            EndpointState e = endpoints[ep];
            if(e.TxBuffer == null)
                e.TxBuffer = new byte[MaxPacketSize];
            if(byteOffset >= 0 && byteOffset < e.TxBuffer.Length)
            {
                e.TxBuffer[byteOffset] = value;
            }
            // Firmware writes bytes sequentially starting at offset 0,
            // so byteOffset+1 IS the byte count after this write. Track
            // the high-water mark to handle out-of-order writes (would
            // be unusual but cheap to support).
            if(byteOffset + 1 > e.TxByteCount)
                e.TxByteCount = byteOffset + 1;
        }

        public override void Reset()
        {
            base.Reset();
            devCtrl = 0;
            usbhsCtrl = 0;
            devEpt = 0;
            devIsrLatched = 0;
            devImr = 0;
            endOfResetFired = false;
            setupInjected = false;
            for(int i = 0; i < EndpointCount; i++)
            {
                endpoints[i].Reset();
            }
            IRQ.Unset();
        }

        public long Size => 0x1000;
        public GPIO IRQ { get; }

        public override Bits StopBits => Bits.One;
        public override Parity ParityBit => Parity.None;
        public override uint BaudRate => 12000000;

        // UARTBase override: the host (pty) is sending bytes to us.
        // Stash them in BULK_OUT's rx buffer, set BYCT + RXOUTI, raise
        // IRQ if RXOUTES is enabled.
        protected override void CharWritten()
        {
            EndpointState ep = endpoints[BULK_OUT];
            // Pre-enumeration: leave bytes queued. The DEVEPTCFG write
            // that flips Allocated=true also calls CharWritten() to
            // pull from the queue once the bulk endpoint exists, so
            // klippy's leading bytes are preserved across the brief
            // window between firmware boot and SET_CONFIGURATION.
            if(!ep.Allocated)
                return;
            // Keep packet→bank delivery monotonic: if a packet is
            // already pending consumption (RXOUTI=1) leave the new
            // bytes queued; we'll re-poll on FIFOCONC after the
            // firmware drains the current bank.
            if((ep.Isr & RXOUTI_BIT) != 0)
                return;
            if(ep.RxBuffer == null)
                ep.RxBuffer = new byte[MaxPacketSize];
            int n = 0;
            byte b;
            while(n < MaxPacketSize && TryGetCharacter(out b))
            {
                ep.RxBuffer[n++] = b;
            }
            if(n > 0)
            {
                ep.RxByteCount = n;
                ep.Isr |= RXOUTI_BIT;
                UpdateIRQ();
            }
        }

        protected override void QueueEmptied()
        {
            // No-op: re-polling happens on every CharWritten and on
            // FIFOCON release.
        }

        private uint ReadDevIsr()
        {
            uint v = devIsrLatched;
            for(int i = 0; i < EndpointCount; i++)
            {
                if((endpoints[i].Imr & endpoints[i].Isr) != 0)
                    v |= (uint)(1 << (DEVISR_PEP_SHIFT + i));
            }
            return v;
        }

        private uint ReadEptIsr(int idx)
        {
            EndpointState ep = endpoints[idx];
            // EP0 SETUP injection: the first time the firmware enables
            // RXSTPES (via DEVEPTIER[0]) and reads DEVEPTISR[0], stage
            // the synthetic SET_CONFIGURATION packet and assert RXSTPI.
            if(idx == 0 && !setupInjected
                && (ep.Imr & RXSTPI_BIT) != 0
                && ep.Allocated)
            {
                StageSetupPacket();
            }
            uint isr = ep.Isr;
            // BYCT is the number of bytes the host has staged. Field
            // width is 11 bits at position 20 (DEVEPTISR.BYCT[30:20]).
            if(ep.RxByteCount > 0)
            {
                isr |= ((uint)ep.RxByteCount & 0x7FF) << 20;
            }
            return isr;
        }

        private void StageSetupPacket()
        {
            // 8-byte USB SETUP for SET_CONFIGURATION(1):
            //   bmRequestType = 0x00 (host-to-device, std, device)
            //   bRequest      = 0x09 (USB_REQ_SET_CONFIGURATION)
            //   wValue        = 0x0001
            //   wIndex        = 0x0000
            //   wLength       = 0x0000
            EndpointState ep = endpoints[0];
            if(ep.RxBuffer == null)
                ep.RxBuffer = new byte[MaxPacketSize];
            ep.RxBuffer[0] = 0x00;
            ep.RxBuffer[1] = 0x09;
            ep.RxBuffer[2] = 0x01; ep.RxBuffer[3] = 0x00;
            ep.RxBuffer[4] = 0x00; ep.RxBuffer[5] = 0x00;
            ep.RxBuffer[6] = 0x00; ep.RxBuffer[7] = 0x00;
            ep.RxByteCount = 8;
            ep.Isr |= RXSTPI_BIT;
            setupInjected = true;
            UpdateIRQ();
        }

        private void ClearEptStatus(int idx, uint mask)
        {
            EndpointState ep = endpoints[idx];
            // Mask is in low bits (TXINI/RXOUTI/RXSTPI/STALLED/...).
            // klipper only ever clears TXINI/RXOUTI/RXSTPI - other
            // bits we silently absorb.
            if((mask & TXINI_BIT) != 0)
            {
                // Firmware just cleared TXINI; that means it has
                // finished writing a packet and the controller would
                // dispatch it on the next IN token. Simulate "host
                // received it instantly":
                //   - Drain the EP's TxBuffer up to TxByteCount and
                //     forward to the UART's TransmitCharacter for the
                //     pty terminal to consume.
                //   - Reset TxByteCount.
                //   - Re-arm TXINI (FIFO is empty again).
                if(idx == BULK_IN && ep.TxBuffer != null
                    && ep.TxByteCount > 0)
                {
                    int len = Math.Min(ep.TxByteCount, ep.TxBuffer.Length);
                    for(int i = 0; i < len; i++)
                    {
                        TransmitCharacter(ep.TxBuffer[i]);
                    }
                }
                ep.TxByteCount = 0;
                // EP0 ZLP after SET_CONFIGURATION: TxByteCount==0 here,
                // we just re-arm TXINI for the next status stage.
                ep.Isr |= TXINI_BIT;
            }
            if((mask & RXOUTI_BIT) != 0)
            {
                // Firmware acknowledged the OUT packet; clear RXOUTI
                // and the byte count. Re-poll the UART RX queue in
                // case more bytes arrived while we were full - this
                // happens routinely under heavy klippy traffic.
                ep.Isr &= ~RXOUTI_BIT;
                ep.RxByteCount = 0;
                if(idx == BULK_OUT)
                {
                    CharWritten();
                }
            }
            if((mask & RXSTPI_BIT) != 0)
            {
                ep.Isr &= ~RXSTPI_BIT;
                ep.RxByteCount = 0;
            }
            UpdateIRQ();
        }

        private void OnFifoCon(int idx)
        {
            // FIFOCONC release: firmware is done with the current FIFO
            // bank. For BULK_OUT, that means we can safely refill from
            // any queued UART bytes; for BULK_IN, the controller would
            // commit the staged TX bank to the host (already done in
            // ClearEptStatus on TXINIC, so nothing extra needed here).
            if(idx == BULK_OUT)
            {
                CharWritten();
            }
        }

        private void UpdateIRQ()
        {
            uint isr = devIsrLatched;
            for(int i = 0; i < EndpointCount; i++)
            {
                if((endpoints[i].Imr & endpoints[i].Isr) != 0)
                    isr |= (uint)(1 << (DEVISR_PEP_SHIFT + i));
            }
            bool active = (isr & devImr) != 0;
            IRQ.Set(active);
        }

        private static bool IsEpRegister(long offset, long baseOffset,
                                         out int idx)
        {
            const int Stride = 4;
            const int Count = EndpointCount;
            if(offset >= baseOffset
                && offset < baseOffset + Stride * Count
                && ((offset - baseOffset) & 3) == 0)
            {
                idx = (int)((offset - baseOffset) / Stride);
                return true;
            }
            idx = -1;
            return false;
        }

        // klipper SAME70 firmware uses USB_CDC_EP_BULK_IN=1, BULK_OUT=2,
        // ACM=3 from src/generic/usb_cdc_ep.h. The model exposes 4 EPs
        // (0..3) since that is all the CDC config descriptor declares;
        // expanding to the chip's full 12-EP capability would only
        // matter for non-CDC firmware variants.
        private const int EndpointCount = 4;
        private const int BULK_IN = 1;
        private const int BULK_OUT = 2;
        private const int MaxPacketSize = 64;
        private const int DEVISR_PEP_SHIFT = 12;
        private const int EpStride = 0x8000;

        private const long EPTCFG_BASE = 0x100;
        private const long EPTISR_BASE = 0x130;
        private const long EPTICR_BASE = 0x160;
        private const long EPTIFR_BASE = 0x190;
        private const long EPTIMR_BASE = 0x1C0;
        private const long EPTIER_BASE = 0x1F0;
        private const long EPTIDR_BASE = 0x220;

        private const uint EPTCFG_ALLOC = 1u << 1;
        private const uint EPT_FIFOCONC = 1u << 14;

        // DEVISR bits we care about. EORST sits at bit 3 (USB end of
        // reset); the firmware enables it via DEVIER bit 3 (EORSTES)
        // - same bit position so the model uses one constant.
        private const uint EORST_BIT = 1u << 3;
        // DEVEPTISR/DEVEPTIER bit positions for TXINI/RXOUTI/RXSTPI.
        private const uint TXINI_BIT  = 1u << 0;
        private const uint RXOUTI_BIT = 1u << 1;
        private const uint RXSTPI_BIT = 1u << 2;

        private uint devCtrl;
        private uint usbhsCtrl;
        private uint devEpt;
        private uint devIsrLatched;
        private uint devImr;
        private bool endOfResetFired;
        private bool setupInjected;

        private readonly EndpointState[] endpoints =
            new EndpointState[EndpointCount];

        private enum Registers
        {
            DEVCTRL    = 0x000,
            DEVISR     = 0x004,
            DEVICR     = 0x008,
            DEVIFR     = 0x00C,
            DEVIMR     = 0x010,
            DEVIDR     = 0x014,
            DEVIER     = 0x018,
            DEVEPT     = 0x01C,
            DEVFNUM    = 0x020,
            USBHS_CTRL = 0x800,
        }

        private class EndpointState
        {
            public uint Cfg;
            public uint Isr;
            public uint Imr;
            public bool Allocated;
            public int TxByteCount;
            public int RxByteCount;
            public byte[] TxBuffer;
            public byte[] RxBuffer;

            public void Reset()
            {
                Cfg = 0;
                Isr = 0;
                Imr = 0;
                Allocated = false;
                TxByteCount = 0;
                RxByteCount = 0;
                TxBuffer = null;
                RxBuffer = null;
            }
        }
    }
}
