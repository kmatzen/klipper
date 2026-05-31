//
// NXP LPC176x A/D converter model for Renode - the IRQ-driven slice
// klipper's src/lpc176x/adc.c actually exercises.
//
// Renode upstream has no LPC176x platform and no LPC176x ADC model
// (search:repo:renode/renode-infrastructure LPC17 returns 0 hits), and
// crucially klipper's LPC ADC driver is INTERRUPT-driven, not polled:
// gpio_adc_sample() arms a burst conversion (ADCR.BURST | ADCR.PDN |
// SEL=1<<chan) and then makes NO further progress until ADC_IRQHandler
// has run five times - each run reads ADDR<chan>, stores (ADDR>>4)&0xfff
// into a 5-entry median filter, and on the 4th turns burst back off.
// gpio_adc_sample() only returns "ready" once adc_status.pos reaches 5,
// which is bumped solely inside the ISR. So a plain register-storeback
// Python stub (afec_stub.py / sam4s_adc_stub.py / stm32_adc_stub.py /
// rp2040_adc_stub.py - all polled) can't drive this peripheral: nothing
// raises the ADC interrupt, the ISR never runs, pos stays 0, and every
// thermistor read times out at temp=0.0. This C# model raises the ADC
// IRQ line (GPIO IRQ -> nvic@22, ADC_IRQn for LPC1768) so the ISR fires.
//
// Loaded into the runtime via `i @<path>/lpc176x_adc.cs` emitted by
// renode_launcher._render_resc before LoadPlatformDescription (same path
// as rp2040_timer.cs / hc32f460_uart.cs), and referenced from
// test/emulator/repl/lpc176x.repl as `adc: Analog.LPC176x_ADC`.
//
// Conversion behaviour: a burst-start write (ADCR with BURST=1, PDN=1,
// SEL!=0) kicks off a fixed-length sequence of conversions on the
// selected channel. Each conversion is *deferred* through a one-shot
// LimitTimer and delivered one at a time, strictly interlocked with the
// firmware's ADDR read: deliver -> set ADDR<chan>.DONE + raise IRQ ->
// ISR reads ADDR<chan> -> clear DONE + drop IRQ + schedule the next
// deliver. This self-pacing (one delivery per ISR read) is what keeps
// the model correct under deterministic tick mode and prevents an
// interrupt storm: we never re-assert the line while a previous DONE is
// still pending, and we stop after the sequence is drained. klipper
// flips burst off mid-sequence (inside the ISR) - we ignore that write,
// since the fixed-length sequence already delivers the five samples the
// median filter needs (plus one drain that the ISR's pos>=5 early-return
// swallows harmlessly).
//
// Conversion values are configured externally by renode_hooks's
// adc_default / adc_set, which - because this is a real C# peripheral,
// not a Python register-storeback stub - drive it through the idiomatic
// FeedSample / SetDefaultValue methods (renode_hooks._iter_adcs resolves
// it as `sysbus.adc`, present only on the LPC platform, so no other
// chip's ADC magic-offset path is touched). FeedSample takes a raw
// 12-bit count (klipper's ADC_MAX=4095); SetDefaultValue takes
// millivolts against a 3300 mV reference, matching the upstream Renode
// ADC convention the hook was written for. Both land in ADDR.RESULT
// (bits [15:4]).

using System;
using Antmicro.Renode.Core;
using Antmicro.Renode.Logging;
using Antmicro.Renode.Peripherals.Bus;
using Antmicro.Renode.Peripherals.Timers; // LimitTimer (rp2040_timer.cs
                                          // gets it implicitly from its
                                          // own namespace; this ADC lives
                                          // in .Analog, so name it)
using Antmicro.Renode.Time;

namespace Antmicro.Renode.Peripherals.Analog
{
    public class LPC176x_ADC : IDoubleWordPeripheral, IKnownSize
    {
        public LPC176x_ADC(IMachine machine)
        {
            IRQ = new GPIO();
            // One-shot, re-armable. Each LimitReached delivers exactly one
            // conversion; the next is armed only when the firmware reads
            // the previous result (see ReadDoubleWord). 1 MHz / limit 1 =
            // a 1 us inter-conversion gap, far shorter than klipper's
            // gpio_adc_sample() retry delay, so the five-sample sequence
            // always completes within a single sample poll window.
            conversionTimer = new LimitTimer(
                machine.ClockSource, 1000000, this, "lpc_adc_conv",
                limit: 1, direction: Direction.Ascending,
                enabled: false, eventEnabled: true,
                workMode: WorkMode.OneShot, autoUpdate: false);
            conversionTimer.LimitReached += DeliverConversion;
            addr = new uint[ChannelCount];
            channelValues = new int[ChannelCount];
            for(int i = 0; i < ChannelCount; i++)
            {
                channelValues[i] = -1; // unset -> fall back to defaultValue
            }
            Reset();
        }

        public uint ReadDoubleWord(long offset)
        {
            switch((Registers)offset)
            {
            case Registers.ADCR:
                return adcr;
            case Registers.ADINTEN:
                return adinten;
            case Registers.ADGDR:
                return adgdr;
            case Registers.ADSTAT:
                return BuildStatus();
            case Registers.ADTRM:
                return adtrm;
            case Registers.ADDR0:
            case Registers.ADDR1:
            case Registers.ADDR2:
            case Registers.ADDR3:
            case Registers.ADDR4:
            case Registers.ADDR5:
            case Registers.ADDR6:
            case Registers.ADDR7:
                return ReadChannelData((int)((offset - (long)Registers.ADDR0) / 4));
            default:
                this.Log(LogLevel.Warning,
                    "LPC176x_ADC: read from unmapped offset 0x{0:X}", offset);
                return 0;
            }
        }

        public void WriteDoubleWord(long offset, uint value)
        {
            switch((Registers)offset)
            {
            case Registers.ADCR:
                adcr = value;
                bool burst = (value & ADCR_BURST) != 0;
                bool powered = (value & ADCR_PDN) != 0;
                uint sel = value & 0xFF;
                // Only a burst-start (BURST|PDN|SEL) kicks off a sequence.
                // The firmware's mid-sequence "burst off" write (ADCR with
                // the channel still selected but BURST cleared) and the
                // gpio_adc_setup / gpio_adc_cancel_sample writes all leave
                // BURST clear, so they don't restart the run.
                if(burst && powered && sel != 0)
                {
                    activeChannel = LowestSetBit(sel);
                    toDeliver = ConversionsPerSample;
                    ScheduleNextConversion();
                }
                break;
            case Registers.ADINTEN:
                adinten = value;
                break;
            case Registers.ADTRM:
                adtrm = value;
                break;
            default:
                this.Log(LogLevel.Warning,
                    "LPC176x_ADC: write 0x{0:X} to unmapped offset 0x{1:X}",
                    value, offset);
                break;
            }
        }

        // renode_hooks.adc_set -> _feed_one(adc, channel, raw): a raw
        // 12-bit conversion count (raw_avr * 4095 // 8184) for one
        // channel. Mirrors the upstream Renode ADC FeedSample signature
        // (value, channelId, repeatCount) so the hook's first attempt
        // binds here.
        public void FeedSample(uint sample, int channelId, int repeatCount)
        {
            if(channelId >= 0 && channelId < ChannelCount)
            {
                channelValues[channelId] = (int)(sample & 0xFFF);
            }
        }

        // renode_hooks.adc_default -> SetDefaultValue(mV, None) sets the
        // all-channel default; the per-channel fallback passes a channel
        // index. Value is millivolts vs a 3300 mV reference (upstream
        // Renode convention), converted back to the 12-bit count ADDR
        // reports.
        public void SetDefaultValue(uint millivolts, int? channel)
        {
            uint counts = (uint)(((int)millivolts * 4095) / 3300) & 0xFFF;
            if(channel.HasValue)
            {
                if(channel.Value >= 0 && channel.Value < ChannelCount)
                {
                    channelValues[channel.Value] = (int)counts;
                }
                return;
            }
            defaultValue = counts;
        }

        public void Reset()
        {
            conversionTimer.Enabled = false;
            adcr = 0;
            adinten = 0;
            adgdr = 0;
            adtrm = 0;
            activeChannel = 0;
            toDeliver = 0;
            for(int i = 0; i < ChannelCount; i++)
            {
                addr[i] = 0;
            }
            // defaultValue / channelValues are intentionally preserved
            // across reset: renode_hooks pushes the fixture's conversion
            // values via FeedSample / SetDefaultValue BEFORE the CPU starts
            // running, so wiping them here (the reset vector runs after)
            // would lose them. They are zero / -1 initialised once in the
            // constructor.
            IRQ.Unset();
        }

        public long Size => 0x40;
        public GPIO IRQ { get; }

        private uint ReadChannelData(int channel)
        {
            uint v = addr[channel];
            if((v & DataDone) != 0)
            {
                // The ISR consumes this result: clear DONE, drop the IRQ
                // line, then pace in the next conversion (if any remain).
                addr[channel] = v & ~DataDone;
                UpdateInterrupt();
                if(channel == activeChannel)
                {
                    ScheduleNextConversion();
                }
            }
            return v;
        }

        private void ScheduleNextConversion()
        {
            if(toDeliver <= 0)
            {
                return;
            }
            toDeliver--;
            conversionTimer.Value = 0;
            conversionTimer.Enabled = true;
        }

        private void DeliverConversion()
        {
            conversionTimer.Enabled = false;
            uint result = (uint)ValueForChannel(activeChannel) & 0xFFF;
            // ADDR layout: DONE(31) | OVERRUN(30) | CHN(26:24) | RESULT(15:4).
            addr[activeChannel] = DataDone
                | ((uint)(activeChannel & 0x7) << 24)
                | (result << 4);
            adgdr = addr[activeChannel];
            UpdateInterrupt();
        }

        private void UpdateInterrupt()
        {
            // ADC interrupt = OR over channels of (ADDR.DONE & ADINTEN).
            // klipper writes ADINTEN=0xff (all channels) so any pending
            // DONE asserts the line; an ADDR read clears that channel's
            // DONE and drops it back.
            bool pending = false;
            for(int i = 0; i < ChannelCount; i++)
            {
                if((addr[i] & DataDone) != 0 && (adinten & (1u << i)) != 0)
                {
                    pending = true;
                    break;
                }
            }
            IRQ.Set(pending);
        }

        private uint BuildStatus()
        {
            // ADSTAT: DONE bits [7:0], OVERRUN bits [15:8], ADINT bit 16.
            uint status = 0;
            bool anyInt = false;
            for(int i = 0; i < ChannelCount; i++)
            {
                if((addr[i] & DataDone) != 0)
                {
                    status |= (1u << i);
                    if((adinten & (1u << i)) != 0)
                    {
                        anyInt = true;
                    }
                }
            }
            if(anyInt)
            {
                status |= (1u << 16);
            }
            return status;
        }

        private int ValueForChannel(int channel)
        {
            return channelValues[channel] >= 0
                ? channelValues[channel] : (int)defaultValue;
        }

        private static int LowestSetBit(uint value)
        {
            for(int i = 0; i < 8; i++)
            {
                if((value & (1u << i)) != 0)
                {
                    return i;
                }
            }
            return 0;
        }

        // Number of conversions delivered per gpio_adc_sample() burst.
        // klipper's median filter needs five samples; ADC_IRQHandler's
        // pos>=5 early return swallows a sixth harmlessly, so deliver six
        // for a one-conversion safety margin against any ISR-entry
        // reordering.
        private const int ConversionsPerSample = 6;
        private const int ChannelCount = 8;

        private const uint ADCR_BURST = 1u << 16;
        private const uint ADCR_PDN = 1u << 21;
        private const uint DataDone = 1u << 31;

        private readonly LimitTimer conversionTimer;
        private readonly uint[] addr;       // ADDR0..ADDR7 live values
        private readonly int[] channelValues; // -1 = use defaultValue
        private uint adcr;
        private uint adinten;
        private uint adgdr;
        private uint adtrm;
        private uint defaultValue;
        private int activeChannel;
        private int toDeliver;

        private enum Registers
        {
            ADCR    = 0x00,
            ADGDR   = 0x04,
            ADINTEN = 0x0C,
            ADDR0   = 0x10,
            ADDR1   = 0x14,
            ADDR2   = 0x18,
            ADDR3   = 0x1C,
            ADDR4   = 0x20,
            ADDR5   = 0x24,
            ADDR6   = 0x28,
            ADDR7   = 0x2C,
            ADSTAT  = 0x30,
            ADTRM   = 0x34,
        }
    }
}
