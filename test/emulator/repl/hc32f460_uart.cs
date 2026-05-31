//
// HC32F460 USART model for Renode.
//
// HDSC HC32F460 has no upstream Renode UART model; the on-chip USART
// register layout (SR/DR/BRR/CR1/CR2/CR3/PR at 32-bit-spaced offsets
// starting at 0x4001D000) doesn't match any of NS16550, STM32_UART,
// SAM_USART, EFM32_UART, etc. Klipper firmware only drives the
// USART through the HDSC vendor library (lib/hc32f460/driver/src/
// hc32f460_usart.c), which restricts the surface to a small subset
// of fields - the model below covers exactly that subset:
//
//   SR  (0x00) [R]   PE/FE/ORE/RXNE/TC/TXE bits read by USART_GetStatus
//   DR  (0x04) [RW]  TDR (bits 0..8) for sends, RDR (bits 16..24) for reads
//   BRR (0x08) [RW]  baud divider - klipper writes via USART_SetBaudrate
//                    but model is real-time and ignores the value
//   CR1 (0x0C) [RW]  RE/TE/RIE/TCIE/TXEIE control + CPE/CFE/CORE
//                    write-1-clear bits for SR error flags
//   CR2 (0x10) [RW]  not driven by klipper - plain storeback
//   CR3 (0x14) [RW]  not driven by klipper - plain storeback
//   PR  (0x18) [RW]  prescaler - klipper writes UsartClkDiv_1 in init
//                    and never reads back - plain storeback
//
// The four IRQ outputs are exposed as named GPIO properties so the
// .repl can wire each to its NVIC line via the
// `<prop> -> nvic@N` syntax (CreationDriver.cs:910-947 reads named
// GPIO properties of type IGPIO). Klipper's src/hc32f460/serial.c
// hardcodes the source -> NVIC line mapping at:
//   serialRx        @ Int000_IRQn  (RIE)
//   serialError     @ Int001_IRQn  (PE/FE/ORE)
//   serialTxEmpty   @ Int002_IRQn  (TXEIE)
//   serialTxComplete@ Int003_IRQn  (TCIE)
// so the .repl wires this peripheral's RxIRQ/ErrorIRQ/TxEmptyIRQ/
// TxCompleteIRQ outputs to nvic@0/1/2/3 respectively. The HC32 INTC
// peripheral (M4_INTC @ 0x40051000) that performs the source->line
// remap on real silicon is left as a passive storeback region in
// the launcher: the firmware writes its source IDs there but never
// reads them back to gate progress, so the static .repl wiring is
// observably equivalent.
//
// Loaded into the Renode runtime via `i @<path>/hc32f460_uart.cs`
// emitted by renode_launcher._render_resc before the platform
// LoadPlatformDescription call. Roslyn-compiles into the running
// Renode process so the .repl's `UART.HC32F460_USART` reference
// resolves at platform-load time.

using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;

namespace Antmicro.Renode.Peripherals.UART
{
    public class HC32F460_USART : UARTBase, IDoubleWordPeripheral, IKnownSize
    {
        public HC32F460_USART(IMachine machine) : base(machine)
        {
            RxIRQ = new GPIO();
            ErrorIRQ = new GPIO();
            TxEmptyIRQ = new GPIO();
            TxCompleteIRQ = new GPIO();

            var registersMap = new Dictionary<long, DoubleWordRegister>
            {
                {(long)Registers.Status, new DoubleWordRegister(this)
                    .WithFlag(0, FieldMode.Read, name: "PE",
                        valueProviderCallback: _ => parityError)
                    .WithFlag(1, FieldMode.Read, name: "FE",
                        valueProviderCallback: _ => frameError)
                    .WithReservedBits(2, 1)
                    .WithFlag(3, FieldMode.Read, name: "ORE",
                        valueProviderCallback: _ => overrunError)
                    .WithReservedBits(4, 1)
                    .WithFlag(5, FieldMode.Read, name: "RXNE",
                        valueProviderCallback: _ => Count > 0)
                    // TC/TXE: model has no FIFO; transmission is
                    // synchronous in WriteDoubleWord. Both stay 1
                    // unconditionally so the firmware's "is the line
                    // free?" polls always succeed.
                    .WithFlag(6, FieldMode.Read, name: "TC",
                        valueProviderCallback: _ => true)
                    .WithFlag(7, FieldMode.Read, name: "TXE",
                        valueProviderCallback: _ => true)
                    .WithReservedBits(8, 24)
                },

                {(long)Registers.Data, new DoubleWordRegister(this)
                    .WithValueField(0, 9, FieldMode.Write, name: "TDR",
                        writeCallback: (_, value) =>
                        {
                            this.TransmitCharacter((byte)(value & 0xff));
                            // TXE/TC stay 1; if TXEIE is enabled the
                            // level-sensitive IRQ re-pends after the
                            // ISR returns and serial_get_tx_byte gets
                            // pulled until the firmware disables
                            // TXEIE itself.
                            UpdateInterrupts();
                        })
                    .WithReservedBits(9, 7)
                    .WithValueField(16, 9, FieldMode.Read, name: "RDR",
                        valueProviderCallback: _ =>
                        {
                            byte ch;
                            if(!TryGetCharacter(out ch))
                            {
                                return 0;
                            }
                            // After consuming, RXNE drops if the
                            // queue is empty - re-evaluate the IRQ
                            // line so the NVIC sees the de-assert.
                            UpdateInterrupts();
                            return ch;
                        })
                    .WithReservedBits(25, 7)
                },

                {(long)Registers.Baud, new DoubleWordRegister(this)
                    .WithValueField(0, 16, name: "BRR")
                    .WithReservedBits(16, 16)
                },

                {(long)Registers.Control1, new DoubleWordRegister(this)
                    .WithFlag(0, name: "RTOE")
                    .WithFlag(1, name: "RTOIE")
                    .WithFlag(2, out rxEnabled, name: "RE",
                        writeCallback: (_, __) => UpdateInterrupts())
                    .WithFlag(3, out txEnabled, name: "TE",
                        writeCallback: (_, __) => UpdateInterrupts())
                    .WithFlag(4, name: "SLME")
                    .WithFlag(5, out rxInterruptEnabled, name: "RIE",
                        writeCallback: (_, __) => UpdateInterrupts())
                    .WithFlag(6, out txCompleteInterruptEnabled,
                        name: "TCIE",
                        writeCallback: (_, __) => UpdateInterrupts())
                    .WithFlag(7, out txEmptyInterruptEnabled,
                        name: "TXEIE",
                        writeCallback: (_, __) => UpdateInterrupts())
                    .WithReservedBits(8, 1)
                    .WithFlag(9, name: "PS")
                    .WithFlag(10, name: "PCE")
                    .WithReservedBits(11, 1)
                    .WithFlag(12, name: "M")
                    .WithReservedBits(13, 2)
                    .WithFlag(15, name: "OVER8")
                    // CPE/CFE/CORE/CRTOF: write-1-to-clear of the
                    // matching SR error bits. Klipper's serialError
                    // ISR uses this path after reading FE/ORE.
                    .WithFlag(16, FieldMode.Write, name: "CPE",
                        writeCallback: (_, v) =>
                        {
                            if(v) parityError = false;
                            UpdateInterrupts();
                        })
                    .WithFlag(17, FieldMode.Write, name: "CFE",
                        writeCallback: (_, v) =>
                        {
                            if(v) frameError = false;
                            UpdateInterrupts();
                        })
                    .WithReservedBits(18, 1)
                    .WithFlag(19, FieldMode.Write, name: "CORE",
                        writeCallback: (_, v) =>
                        {
                            if(v) overrunError = false;
                            UpdateInterrupts();
                        })
                    .WithFlag(20, FieldMode.Write, name: "CRTOF")
                    .WithReservedBits(21, 3)
                    .WithFlag(24, name: "MS")
                    .WithReservedBits(25, 3)
                    .WithFlag(28, name: "ML")
                    .WithFlag(29, name: "FBME")
                    .WithFlag(30, name: "NFE")
                    .WithFlag(31, name: "SBS")
                },

                {(long)Registers.Control2, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "CR2")},
                {(long)Registers.Control3, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "CR3")},
                {(long)Registers.Prescaler, new DoubleWordRegister(this)
                    .WithValueField(0, 32, name: "PR")},
            };

            registers = new DoubleWordRegisterCollection(this, registersMap);
        }

        public uint ReadDoubleWord(long offset) => registers.Read(offset);
        public void WriteDoubleWord(long offset, uint value) => registers.Write(offset, value);

        public override void Reset()
        {
            base.Reset();
            registers.Reset();
            parityError = false;
            frameError = false;
            overrunError = false;
            UpdateInterrupts();
        }

        public long Size => 0x20;
        public override Bits StopBits => Bits.One;
        public override Parity ParityBit => Parity.None;
        public override uint BaudRate => 115200;

        public GPIO RxIRQ { get; }
        public GPIO ErrorIRQ { get; }
        public GPIO TxEmptyIRQ { get; }
        public GPIO TxCompleteIRQ { get; }

        protected override void CharWritten() => UpdateInterrupts();
        protected override void QueueEmptied() => UpdateInterrupts();

        private void UpdateInterrupts()
        {
            // Level-sensitive: NVIC re-pends after each ISR exit
            // while the line stays asserted. RX line tracks RXNE &&
            // RIE; TXE/TC lines track only their respective
            // interrupt-enable bits because the underlying status
            // bits are wired-true in this model.
            RxIRQ.Set(rxInterruptEnabled.Value && Count > 0);
            ErrorIRQ.Set(parityError || frameError || overrunError);
            TxEmptyIRQ.Set(txEmptyInterruptEnabled.Value);
            TxCompleteIRQ.Set(txCompleteInterruptEnabled.Value);
        }

        private bool parityError;
        private bool frameError;
        private bool overrunError;

        private IFlagRegisterField rxEnabled;
        private IFlagRegisterField txEnabled;
        private IFlagRegisterField rxInterruptEnabled;
        private IFlagRegisterField txEmptyInterruptEnabled;
        private IFlagRegisterField txCompleteInterruptEnabled;

        private readonly DoubleWordRegisterCollection registers;

        private enum Registers
        {
            Status   = 0x00,
            Data     = 0x04,
            Baud     = 0x08,
            Control1 = 0x0C,
            Control2 = 0x10,
            Control3 = 0x14,
            Prescaler = 0x18,
        }
    }
}
