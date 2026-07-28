//
// RP2040 64-bit / 1 MHz timer model for Renode.
//
// The RP2040 timer at 0x40054000 is a 64-bit upcounter clocked at 1 MHz
// with four 32-bit alarm comparators. klipper firmware (src/rp2040/
// timer.c) only uses ALARM0 + the bit 0 lanes of INTR / INTE / INTS /
// ARMED, plus TIMERAWL for clock readout. The full 4-alarm surface is
// modelled anyway because the cost is negligible and other paths
// (e.g. armcm_timer.c on RP2350 - though klipper RP2350 uses the
// generic timer instead, the SDK convention covers all four) might
// reach for ALARM1..3.
//
// Renode upstream has no RP2040-family timer model in
// renode-infrastructure (search:repo:renode/renode-infrastructure
// rp2040 returns no timer hits as of 1.16.x); the closest analogues
// are LimitTimer-driven peripherals like Antmicro.Renode.Peripherals.
// Timers.STM32_Timer, which doesn't fit because RP2040 alarms are
// absolute-target compare against the free-running counter rather
// than period reload.
//
// Loaded into the runtime via `i @<path>/rp2040_timer.cs` emitted by
// renode_launcher._render_resc before LoadPlatformDescription, same
// path as test/emulator/repl/hc32f460_uart.cs.

using System;
using System.Collections.Generic;
using Antmicro.Renode.Core;
using Antmicro.Renode.Core.Structure.Registers;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Timers
{
    public class RP2040_Timer : IDoubleWordPeripheral, IKnownSize
    {
        public RP2040_Timer(IMachine machine)
        {
            this.machine = machine;
            IRQ = new GPIO();

            // Free-running 1 MHz counter. eventEnabled=false because we
            // never need a "limit reached" callback on the wraparound -
            // the counter is consulted only by sampled reads.
            counter = new LimitTimer(
                machine.ClockSource, 1000000, this, "rp2040_counter",
                limit: ulong.MaxValue, direction: Direction.Ascending,
                enabled: true, eventEnabled: false,
                workMode: WorkMode.Periodic, autoUpdate: true);

            // Per-alarm one-shot timers. Each one's LimitReached fires
            // the matching INTR bit when virtual time crosses the
            // 32-bit ALARM<n> compare value.
            for(int i = 0; i < AlarmCount; i++)
            {
                int idx = i; // local copy for closure capture
                alarmTimers[i] = new LimitTimer(
                    machine.ClockSource, 1000000, this, "rp2040_alarm" + idx,
                    limit: 1, direction: Direction.Ascending,
                    enabled: false, eventEnabled: true,
                    workMode: WorkMode.OneShot, autoUpdate: false);
                alarmTimers[i].LimitReached += () => OnAlarmReached(idx);
            }
        }

        public uint ReadDoubleWord(long offset)
        {
            switch((Registers)offset)
            {
            case Registers.TIMEHW: // returns latched high word from TIMELR/HR pair
                return latchedHigh;
            case Registers.TIMELW:
                return latchedLow;
            case Registers.TIMEHR:
                return (uint)(counter.Value >> 32);
            case Registers.TIMELR:
                LatchLowHigh();
                return latchedLow;
            case Registers.ALARM0: return alarms[0];
            case Registers.ALARM1: return alarms[1];
            case Registers.ALARM2: return alarms[2];
            case Registers.ALARM3: return alarms[3];
            case Registers.ARMED:  return armed;
            case Registers.TIMERAWH: return (uint)(counter.Value >> 32);
            case Registers.TIMERAWL: return (uint)counter.Value;
            case Registers.DBGPAUSE: return dbgpause;
            case Registers.PAUSE:    return pause;
            case Registers.INTR: return intr;
            case Registers.INTE: return inte;
            case Registers.INTF: return intf;
            case Registers.INTS: return (intr | intf) & inte;
            default:
                this.Log(LogLevel.Warning,
                    "RP2040_Timer: read from unmapped offset 0x{0:X}",
                    offset);
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch((Registers)offset)
            {
            case Registers.TIMEHW:
                // The SDK convention is "write TIMELW first, then
                // TIMEHW commits the 64-bit load". klipper firmware
                // (timer_init) writes 0 to both at startup, which is a
                // no-op against a free-running counter that already
                // started at 0. Any other write rebases the counter
                // from the staged TIMELW + the TIMEHW just written.
                ulong newValue = ((ulong)value << 32) | pendingLow;
                counter.Value = newValue;
                pendingLow = 0;
                break;
            case Registers.TIMELW:
                pendingLow = value;
                break;
            case Registers.ALARM0: ArmAlarm(0, value); break;
            case Registers.ALARM1: ArmAlarm(1, value); break;
            case Registers.ALARM2: ArmAlarm(2, value); break;
            case Registers.ALARM3: ArmAlarm(3, value); break;
            case Registers.ARMED:
                // W1C: writing 1 disarms (cancels) the matching alarm
                // without firing its IRQ.
                for(int i = 0; i < AlarmCount; i++)
                {
                    if((value & (1u << i)) != 0)
                    {
                        armed &= ~(1u << i);
                        alarmTimers[i].Enabled = false;
                    }
                }
                break;
            case Registers.DBGPAUSE: dbgpause = value; break;
            case Registers.PAUSE:    pause = value; break;
            case Registers.INTR:
                // W1C of latched-and-forced alarm bits.
                intr &= ~value;
                UpdateIRQ();
                break;
            case Registers.INTE:
                inte = value & 0xFu;
                UpdateIRQ();
                break;
            case Registers.INTF:
                // Force-fire bits (test path); klipper doesn't drive
                // these but the surface is cheap to model.
                intf = value & 0xFu;
                UpdateIRQ();
                break;
            default:
                this.Log(LogLevel.Warning,
                    "RP2040_Timer: write 0x{0:X} to unmapped offset 0x{1:X}",
                    value, offset);
                break;
            }
        }

        public void Reset()
        {
            counter.Reset();
            counter.Enabled = true;
            for(int i = 0; i < AlarmCount; i++)
            {
                alarmTimers[i].Enabled = false;
                alarms[i] = 0;
            }
            armed = 0;
            intr = 0;
            inte = 0;
            intf = 0;
            dbgpause = 0;
            pause = 0;
            pendingLow = 0;
            latchedLow = 0;
            latchedHigh = 0;
            IRQ.Unset();
        }

        public long Size => 0x44;
        public GPIO IRQ { get; }

        // ALARM<n> write semantics (datasheet 4.6.4): writing arms the
        // alarm; on real silicon a match against TIMERAWL latches
        // INTR.<n>. We implement match-detection by scheduling a
        // one-shot LimitTimer for (alarm - now) microseconds. If
        // alarm < now (delta wraps to a huge unsigned value), real
        // silicon would wait for the 32-bit counter to wrap around
        // ~71 minutes later; we honor that semantically by scheduling
        // the full delta, but cap it at 1 us minimum so a write of
        // "ALARM == now" still fires next tick rather than getting
        // dropped on the floor.
        private void ArmAlarm(int idx, uint targetMicros)
        {
            alarms[idx] = targetMicros;
            uint nowLow = (uint)counter.Value;
            uint delta = unchecked(targetMicros - nowLow);
            if(delta == 0)
            {
                delta = 1;
            }
            armed |= (1u << idx);
            alarmTimers[idx].Limit = delta;
            alarmTimers[idx].Value = 0;
            alarmTimers[idx].Enabled = true;
        }

        private void OnAlarmReached(int idx)
        {
            armed &= ~(1u << idx);
            intr |= (1u << idx);
            alarmTimers[idx].Enabled = false;
            UpdateIRQ();
        }

        private void UpdateIRQ()
        {
            // Bit 0 of (INTR | INTF) & INTE corresponds to ALARM_IRQ_0,
            // which klipper wires to TIMER_IRQ_0_IRQn. Higher alarm
            // bits would route to separate NVIC lines on real silicon
            // (alarm1 -> IRQ1, alarm2 -> IRQ2, alarm3 -> IRQ3); klipper
            // only uses ALARM0 so the model exposes a single combined
            // GPIO whose level tracks bit 0 of INTS. If a future
            // klipper variant uses multiple alarms we'd extend this to
            // an INumberedGPIOOutput with one connection per alarm.
            uint ints = (intr | intf) & inte;
            IRQ.Set((ints & 1u) != 0);
        }

        private void LatchLowHigh()
        {
            ulong v = counter.Value;
            latchedLow = (uint)v;
            latchedHigh = (uint)(v >> 32);
        }

        private const int AlarmCount = 4;

        private readonly IMachine machine;
        private readonly LimitTimer counter;
        private readonly LimitTimer[] alarmTimers = new LimitTimer[AlarmCount];
        private readonly uint[] alarms = new uint[AlarmCount];
        private uint armed;
        private uint intr;
        private uint inte;
        private uint intf;
        private uint dbgpause;
        private uint pause;
        private uint pendingLow;
        private uint latchedLow;
        private uint latchedHigh;

        private enum Registers
        {
            TIMEHW   = 0x00,
            TIMELW   = 0x04,
            TIMEHR   = 0x08,
            TIMELR   = 0x0C,
            ALARM0   = 0x10,
            ALARM1   = 0x14,
            ALARM2   = 0x18,
            ALARM3   = 0x1C,
            ARMED    = 0x20,
            TIMERAWH = 0x24,
            TIMERAWL = 0x28,
            DBGPAUSE = 0x2C,
            PAUSE    = 0x30,
            INTR     = 0x34,
            INTE     = 0x38,
            INTF     = 0x3C,
            INTS     = 0x40,
        }
    }
}
