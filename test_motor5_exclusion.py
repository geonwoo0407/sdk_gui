"""Exercise ID 5 exclusion with real GUI methods and simulated motor buses."""
import copy
from types import SimpleNamespace, MethodType
import unittest
from unittest.mock import Mock, patch

from dynamixel_sdk import COMM_SUCCESS, DXL_LOBYTE, DXL_HIBYTE, DXL_LOWORD, DXL_HIWORD
from PyQt5.QtWidgets import QApplication, QCheckBox, QPushButton
import test_sdk_gui_composer_timeline as gui
import test_sdk_gui_timed_playback as playback


class Writer:
    def __init__(self):
        self.params = {}
        self.sent = []

    def clearParam(self):
        self.params = {}

    def addParam(self, joint, data):
        self.params[joint] = list(data)
        return True

    def txPacket(self):
        self.sent.append(copy.deepcopy(self.params))
        return COMM_SUCCESS


class Reader:
    def __init__(self):
        self.ids = []
        self.requests = []

    def clearParam(self):
        self.ids = []

    def addParam(self, joint):
        self.ids.append(joint)
        return True

    def txRxPacket(self):
        self.requests.append(self.ids.copy())
        # Reproduce a whole SyncRead failure if the unresponsive ID is included.
        return -3001 if 5 in self.ids else COMM_SUCCESS

    def isAvailable(self, joint, address, length):
        return joint in self.ids

    def getData(self, joint, address, length):
        return 2048


class ExclusionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.warnings = []
        self.addCleanup(patch.stopall)
        patch.dict(gui.namespace, {
            'COMM_SUCCESS': COMM_SUCCESS, 'DXL_LOBYTE': DXL_LOBYTE,
            'DXL_HIBYTE': DXL_HIBYTE, 'DXL_LOWORD': DXL_LOWORD,
            'DXL_HIWORD': DXL_HIWORD,
            'time': SimpleNamespace(perf_counter=lambda: 0.0, sleep=lambda value: None),
            'QMessageBox': SimpleNamespace(
                warning=lambda *args: self.warnings.append(args[-1]),
                information=lambda *args: None),
        }).start()
        self.p = p = gui.namespace['Editor']()
        self.addCleanup(p.close)
        p.port_opened = True
        p.is_playing = p.is_paused = p.robot_sync_enabled = False
        p.frame_apply_timer = Mock()
        p.frame_apply_timer.isActive.return_value = False
        p.excluded_joint_ids = {5}
        p.online_joints = [4, 6]
        p.joints = {4: 10.0, 5: 20.0, 6: 30.0}
        p.feedback_angles = p.joints.copy()
        p.commanded_angles = p.joints.copy()
        p.commanded_joint_ids = {4, 6}
        p.time_based_profile_ids = {4, 6}
        p.motion_sequence = [dict(name='walk', start_ms=0, time_ms=100,
                                  angles={'4': 11.0, '5': 21.0, '6': 31.0})]
        p.torque_btns = {joint: QPushButton() for joint in (4, 5, 6)}
        for button in p.torque_btns.values():
            button.setCheckable(True)
            button.setChecked(True)
            self.addCleanup(button.close)
        p.sliders = {j: Mock() for j in (4, 5, 6)}
        p.spinboxes = {j: Mock() for j in (4, 5, 6)}
        p.torque_groups = {'arm': [4, 5, 6]}
        p.torque_group_btns = {'arm': QPushButton()}
        self.addCleanup(p.torque_group_btns['arm'].close)
        p.torque_group_btns['arm'].setCheckable(True)
        p.check_exclude_motor5 = QCheckBox()
        p.check_exclude_motor5.setChecked(True)
        p.check_exclude_motor5.toggled.connect(p.set_motor5_excluded)
        self.addCleanup(p.check_exclude_motor5.close)
        p.groupSyncWrite = Writer()
        p.groupSyncWriteTorque = Writer()
        p.groupSyncRead = Reader()
        p.packetHandler = Mock()
        p.portHandler = Mock()
        p.update_joint_display = lambda j, value, **kw: p.joints.update({j: value})
        p.update_3d_robot = Mock()
        p.configure_unlimited_motor_profiles = Mock(return_value=True)
        p.configure_time_based_drive_mode = Mock(return_value=True)

    def test_scan_does_not_request_id5(self):
        p = self.p
        p.packetHandler.ping.return_value = (30, COMM_SUCCESS, 0)
        p.detect_online_joints()
        self.assertNotIn(5, p.online_joints)
        self.assertEqual(len(p.online_joints), 22)
        self.assertEqual(p.packetHandler.ping.call_count, 22)

    def test_prepare_skips_id5_but_keeps_other_torque_checks(self):
        p = self.p
        p.detect_online_joints = Mock()
        checked = []
        p.read_torque_enabled = lambda j: checked.append(j) or True
        self.assertTrue(p.prepare_real_robot_sequence())
        self.assertEqual(set(checked), {4, 6})
        self.assertEqual(p.sequence_joint_ids(), {4, 6})
        self.assertEqual(self.warnings, [])

    def test_other_unknown_torque_still_blocks(self):
        p = self.p
        p.detect_online_joints = Mock()
        p.read_torque_enabled = lambda j: None if j == 4 else True
        self.assertFalse(p.prepare_real_robot_sequence())
        self.assertIn('[4]', self.warnings[-1])

    def test_default_without_exclusion_still_blocks_missing_id5(self):
        p = self.p
        p.excluded_joint_ids = set()
        p.detect_online_joints = Mock()
        self.assertFalse(p.prepare_real_robot_sequence())
        self.assertIn('[5]', self.warnings[-1])

    def test_playback_feedback_reads_other_motors_even_with_stale_id5_list(self):
        p = self.p
        p.online_joints = [4, 5, 6]
        p.fk_feedback_sample_count = 0
        p.calculate_fk_tracking_error = Mock(return_value={})
        p.print_fk_tracking_error = Mock()
        self.assertTrue(p.read_feedback_and_report_fk())
        self.assertEqual(p.groupSyncRead.requests, [[4, 6]])
        self.assertEqual(p.fk_feedback_sample_count, 1)

    def test_other_confirmed_off_torque_still_blocks(self):
        p = self.p
        p.detect_online_joints = Mock()
        p.read_torque_enabled = lambda j: False if j == 4 else True
        p.set_all_torque_on = Mock()
        self.assertFalse(p.prepare_real_robot_sequence())
        p.set_all_torque_on.assert_called_once()
        self.assertIn('OFF: [4]', self.warnings[-1])

    def test_excluded_read_returns_unknown_instead_of_fabricating_on(self):
        p = self.p
        self.assertIsNone(p.read_torque_enabled(5))
        self.assertIsNone(p.read_present_angle(5))
        self.assertEqual(p.packetHandler.mock_calls, [])

    def test_goal_transmission_and_tracking_exclude_id5(self):
        p = self.p
        angles = {4: 11.0, 5: 21.0, 6: 31.0}
        self.assertTrue(p.write_goal_positions(angles, target_ids=[4, 5, 6], read_feedback=False))
        self.assertEqual(set(p.groupSyncWrite.sent[-1]), {4, 6})
        p.set_tracking_target_angles(angles)
        self.assertNotIn(5, p.commanded_joint_ids)
        self.assertEqual(p.commanded_angles[5], 20.0)

    def test_only_excluded_frame_does_not_send_or_stop_other_frames(self):
        p = self.p
        self.assertTrue(p.write_goal_positions({5: 90.0}, target_ids=[], read_feedback=False))
        self.assertEqual(p.groupSyncWrite.sent, [])
        p.timed_gate_frame = dict(name='arm only', start_ms=0, time_ms=100, angles={'5': 90})
        p.fail_timed_playback = Mock()
        self.assertEqual(p.gate_timeline_on_frame_arrival(100, 0), 100)
        p.fail_timed_playback.assert_not_called()
        trial = playback.player()
        trial.port_opened = True
        trial.online_joints = {4}
        trial.excluded_joint_ids = {5}
        trial.groupSyncWrite = Writer()
        trial.write_goal_positions = MethodType(gui.namespace['Editor'].write_goal_positions, trial)
        playback.set_frames(trial, [dict(name='arm only', start_ms=0, time_ms=100,
                                        angles={'5': 90}),
                                   dict(name='leg', start_ms=100, time_ms=100,
                                        angles={'4': 10})])
        playback.tick(trial, 0)
        self.assertTrue(trial.is_playing)
        self.assertEqual(trial.errors, [])
        self.assertEqual(trial.groupSyncWrite.sent, [])

    def test_all_torque_on_reads_and_writes_other_motors_only(self):
        p = self.p
        p.online_joints = [4, 5, 6]  # Even a stale list must not include ID 5.
        p.set_all_torque_on()
        self.assertEqual(p.groupSyncRead.requests, [[4, 6]])
        self.assertEqual(set(p.groupSyncWrite.sent[-1]), {4, 6})
        self.assertEqual(p.groupSyncWriteTorque.sent[-1], {4: [1], 6: [1]})
        self.assertEqual(p.torque_btns[5].text(), '제외')
        self.assertFalse(p.torque_btns[5].isChecked())
        self.assertEqual(self.warnings, [])

    def test_all_torque_off_still_addresses_excluded_id5(self):
        p = self.p
        p.set_all_torque_off()
        self.assertEqual(p.groupSyncWriteTorque.sent[-1], {4: [0], 5: [0], 6: [0]})
        self.assertEqual(p.torque_btns[5].text(), '제외')
        p.configure_time_based_drive_mode.assert_called_once_with([4, 6])

    def test_checkbox_does_not_send_any_command_or_modify_saved_angles(self):
        p = self.p
        p.excluded_joint_ids = set()
        p.online_joints = [4, 5, 6]
        p.commanded_joint_ids.add(5)
        before = copy.deepcopy(p.motion_sequence)
        p.set_motor5_excluded(True)
        self.assertEqual(p.online_joints, [4, 6])
        self.assertEqual(p.motion_sequence, before)
        self.assertEqual(p.packetHandler.mock_calls, [])
        self.assertEqual(p.groupSyncWriteTorque.sent, [])
        self.assertEqual(p.torque_btns[5].text(), '제외')
        p.set_motor5_excluded(False)
        self.assertNotIn(5, p.online_joints)  # Requires a real rescan, not fake online.
        self.assertEqual(p.excluded_joint_ids, set())

    def test_checkbox_cannot_change_during_playback_or_sync(self):
        p = self.p
        for field in ('is_playing', 'is_paused', 'robot_sync_enabled'):
            setattr(p, field, True)
            p.set_motor5_excluded(False)
            self.assertEqual(p.excluded_joint_ids, {5})
            self.assertTrue(p.check_exclude_motor5.isChecked())
            setattr(p, field, False)
        p.frame_apply_timer.isActive.return_value = True
        p.set_motor5_excluded(False)
        self.assertEqual(p.excluded_joint_ids, {5})

    def test_single_frame_application_ignores_id5(self):
        p = self.p
        p.frames = p.motion_sequence
        p.selected_library_frame_index = lambda: 0
        p.time_spinbox = Mock()
        read = []
        p.read_present_angle = lambda j: read.append(j) or p.joints[j]
        p.apply_selected_frame()
        self.assertEqual(read, [4, 6])
        self.assertEqual(set(p.frame_apply_target_angles), {4, 6})
        self.assertEqual(self.warnings, [])


if __name__ == '__main__':
    unittest.main()
