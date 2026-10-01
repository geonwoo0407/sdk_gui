"""Decode captured/fake registers without opening a serial port."""
import unittest

from diagnose_mx_bus import decode_config, decode_ram


class MotorDiagnosticsTests(unittest.TestCase):
    def test_mx28_reports_shutdown_and_load_without_inventing_current(self):
        ram = bytearray(83)
        ram[70 - 64] = 0x24
        ram[126 - 64:128 - 64] = (-500).to_bytes(2, 'little', signed=True)
        ram[144 - 64:146 - 64] = (123).to_bytes(2, 'little')
        ram[146 - 64] = 81
        state = decode_ram(ram, 30)
        self.assertEqual(state['torque'], 0)
        self.assertEqual(state['faults'], ['overheating', 'overload'])
        self.assertEqual(state['present_load_percent'], -50.0)
        self.assertEqual(state['input_voltage_v'], 12.3)
        self.assertEqual(state['temperature_c'], 81)
        self.assertNotIn('present_current_ma', state)
        self.assertNotIn('goal_current_raw', state)
        config = bytearray(64)
        config[:2] = (30).to_bytes(2, 'little')
        config[38:40] = b'\xff\xff'  # Reserved on MX-28.
        self.assertNotIn('current_limit_raw', decode_config(config))

    def test_mx106_current_decoding_is_preserved(self):
        ram = bytearray(83)
        ram[126 - 64:128 - 64] = (-100).to_bytes(2, 'little', signed=True)
        ram[102 - 64:104 - 64] = (2047).to_bytes(2, 'little')
        state = decode_ram(ram)
        self.assertEqual(state['present_current_ma'], -336.0)
        self.assertEqual(state['goal_current_raw'], 2047)
        self.assertNotIn('present_load_percent', state)
        config = bytearray(64)
        config[:2] = (321).to_bytes(2, 'little')
        config[38:40] = (2047).to_bytes(2, 'little')
        self.assertEqual(decode_config(config)['current_limit_raw'], 2047)

    def test_unknown_or_incomplete_registers_are_rejected(self):
        for data, model in [(bytearray(83), 999), (bytearray(82), 30)]:
            with self.assertRaises(ValueError):
                decode_ram(data, model)
        for config in [bytearray(64), bytearray(63)]:
            with self.assertRaises(ValueError):
                decode_config(config)


if __name__ == '__main__':
    unittest.main()
