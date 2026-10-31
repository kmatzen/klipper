# Regression tests for clock polling after an MCU stops responding.
# Run with: python3 -m unittest discover -s test/unit -p test_clocksync.py
# This file may be distributed under the terms of the GNU GPLv3 license.
import pathlib
import sys
import tempfile
import unittest

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2] / 'klippy'))
import clocksync


class Reactor:
    def register_timer(self, callback):
        return callback


class Serial:
    def __init__(self):
        self.sent = []

    def raw_send(self, *args):
        self.sent.append(args)


class ClockPollingTest(unittest.TestCase):
    def make_sync(self):
        sync = clocksync.ClockSync(Reactor())
        sync.serial = Serial()
        sync.get_clock_cmd = b'\x01'
        sync.cmd_queue = object()
        return sync

    def test_unresponsive_mcu_has_bounded_queries(self):
        sync = self.make_sync()
        for i in range(10000):
            self.assertEqual(sync._get_clock_event(float(i)), i + .9839)
        self.assertEqual(len(sync.serial.sent), 5)
        self.assertEqual(sync.queries_pending, 5)
        self.assertFalse(sync.is_active())

    def test_timeout_threshold_is_unchanged(self):
        sync = self.make_sync()
        for i in range(5):
            self.assertTrue(sync.is_active())
            sync._get_clock_event(float(i))
        self.assertFalse(sync.is_active())

    def test_late_response_resumes_polling(self):
        sync = self.make_sync()
        for i in range(20):
            sync._get_clock_event(float(i))
        # A retransmitted clock response has no sent_time. It still resets
        # the pending count, without changing the regression estimate.
        sync._handle_clock({'clock': 123, '#sent_time': 0.,
                            '#receive_time': 21.})
        self.assertTrue(sync.is_active())
        before = len(sync.serial.sent)
        sync._get_clock_event(22.)
        self.assertEqual(len(sync.serial.sent), before + 1)
        self.assertEqual(sync.queries_pending, 1)

    def test_healthy_mcu_keeps_polling(self):
        sync = self.make_sync()
        for i in range(100):
            sync._get_clock_event(float(i))
            sync._handle_clock({'clock': i, '#sent_time': 0.,
                                '#receive_time': float(i)})
        self.assertEqual(len(sync.serial.sent), 100)
        self.assertTrue(sync.is_active())

    def test_secondary_mcu_has_independent_limit(self):
        primary = self.make_sync()
        secondary = clocksync.SecondarySync(Reactor(), primary)
        secondary.serial = Serial()
        for i in range(20):
            primary._get_clock_event(float(i))
            secondary._get_clock_event(float(i))
            secondary._handle_clock({'clock': i, '#sent_time': 0.,
                                     '#receive_time': float(i)})
        self.assertEqual(len(primary.serial.sent), 5)
        self.assertEqual(len(secondary.serial.sent), 20)

    def test_stopped_native_queue_has_bounded_growth(self):
        import chelper
        ffi, lib = chelper.get_ffi()
        with tempfile.TemporaryFile() as output:
            sq = lib.serialqueue_alloc(output.fileno(), b'f', 0, b'test')
            cq = lib.serialqueue_alloc_commandqueue()
            # Model the serial worker exiting after a device read failure.
            lib.serialqueue_exit(sq)
            try:
                class NativeSerial:
                    def raw_send(self, cmd, minclock, reqclock, queue):
                        lib.serialqueue_send(sq, queue, cmd, len(cmd),
                                             minclock, reqclock, 0)

                sync = self.make_sync()
                sync.serial = NativeSerial()
                sync.cmd_queue = cq
                for i in range(10000):
                    sync._get_clock_event(float(i))
                stats = ffi.new('char[4096]')
                lib.serialqueue_get_stats(sq, stats, len(stats))
                fields = dict(item.split('=') for item in
                              ffi.string(stats).decode().split())
                self.assertEqual(int(fields['upcoming_bytes']), 5)
                self.assertEqual(int(fields['bytes_write']), 0)
            finally:
                lib.serialqueue_free(sq)
                lib.serialqueue_free_commandqueue(cq)


if __name__ == '__main__':
    unittest.main()
