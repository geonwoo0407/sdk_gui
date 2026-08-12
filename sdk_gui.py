import sys
import math
import fractions
import time
import copy
import json
import os
import signal
import struct
import uuid
import xml.etree.ElementTree as ET
# 아무거나나ㅏㅏㅏ

# 🚨 [필수] 파이썬 3.9+ fractions.gcd -> math.gcd 긴급 호환성 패치
fractions.gcd = math.gcd

# 🚨 [필수] 파이썬 3.10+ collections 호환성 완벽 끝판왕 패치
import collections
import collections.abc
for name in dir(collections.abc):
    if not name.startswith('_'):
        setattr(collections, name, getattr(collections.abc, name))

from PyQt5.QtWidgets import (QApplication, QWidget, QVBoxLayout, QHBoxLayout, 
                             QSlider, QLabel, QPushButton, QListWidget, QFileDialog, 
                             QSpinBox, QDoubleSpinBox, QGroupBox, QScrollArea, QFrame, QInputDialog,
                             QTabWidget, QMessageBox, QGridLayout, QCheckBox, QListWidgetItem, QAbstractItemView,
                             QOpenGLWidget, QShortcut, QComboBox)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QPainter, QColor, QPen, QMatrix4x4, QKeySequence, QPainterPath

# 🔥 다이나믹셀 SDK 라이브러리 임포트
from dynamixel_sdk import *
from dynamixel_sdk.protocol2_packet_handler import Protocol2PacketHandler

# 🤖 하드웨어 및 다이나믹셀 통신 주소 세팅 (MX 시리즈 프로토콜 2.0 기준)
ADDR_FIRMWARE_VERSION       = 6
ADDR_DRIVE_MODE             = 10
ADDR_TORQUE_ENABLE          = 64
ADDR_PROFILE_ACCELERATION   = 108
ADDR_PROFILE_VELOCITY       = 112
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
LEN_PROFILE_VALUE           = 4
LEN_GOAL_POSITION           = 4         
LEN_PRESENT_POSITION        = 4         
PROTOCOL_VERSION            = 2.0       
BAUDRATE                    = 4000000   
DEVICENAME                  = '/dev/ttyUSB0' # (자동 탐색기가 완전히 실패했을 때의 최후의 백업 포트)

# MX(2.0) 펌웨어 V42+에서는 Drive Mode bit 2를 켜면 Profile Velocity가
# 속도 제한값이 아니라 Goal Position 도착시간(ms)이 됩니다. 프레임마다
# 이 시간을 지정하고 최종 각도를 한 번만 전송해 모터 내부 제어기가 해당
# 시간에 맞춰 속도를 자동으로 정하도록 합니다.
DRIVE_MODE_TIME_BASED_BIT   = 0x04
MIN_TIME_PROFILE_FIRMWARE   = 42
MAX_TIME_PROFILE_MS         = 32737
LANDING_FRAME_TAG           = "[착지]"
LANDING_ACCEL_RATIO         = 0.10
LANDING_ACCEL_MAX_MS        = 30
# 발 들기 프레임은 전체 시간의 앞 80%에 목표각까지 도달 명령을 보내고,
# 남은 20% 동안 최종 Goal을 유지해 실제 모터가 따라붙을 시간을 줍니다.
LIFT_TARGET_ARRIVAL_RATIO   = 0.80
LIFT_FRAME_KEYWORDS         = ("발들", "들기", "오들", "왼들")
TIMED_FEEDBACK_FREQUENCY_HZ = 100
TIMED_FEEDBACK_INTERVAL_SEC = 1.0 / TIMED_FEEDBACK_FREQUENCY_HZ
FRAME_REACHED_TOLERANCE_DEG = 2.0
FRAME_REACHED_TIMEOUT_SEC   = 3.0
FRAME_GATE_CHECK_SEC        = 0.02

# 타임라인 목표를 200Hz로 평가합니다. GUI 실기 시퀀스도 같은 주기로
# half-cosine 중간 Goal을 전송하고, 프레임 끝에서는 최종각을 다시 전송한 뒤
# 추가 대기 없이 다음 tick부터 다음 프레임 궤적을 시작합니다.
# Qt 이벤트 루프는 실시간 스케줄러가 아니므로 실제 주기에 지터는 있을 수
# 있지만, 모노토닉 시간을 사용해 누적 타임라인 오차가 생기지 않게 합니다.
CONTROL_FREQUENCY_HZ        = 200
CONTROL_INTERVAL_MS         = int(round(1000.0 / CONTROL_FREQUENCY_HZ))
PLAYBACK_UI_INTERVAL_SEC    = 1.0 / 30.0
# 피드백/FK는 시간 프로파일을 방해하지 않도록 100Hz에서 읽고, 터미널 I/O는
# 제어 루프를 지연시키지 않도록 요약 출력만 10Hz로 제한합니다.
FK_ERROR_LOG_FREQUENCY_HZ   = 10
FK_ERROR_LOG_INTERVAL_SEC   = 1.0 / FK_ERROR_LOG_FREQUENCY_HZ

# DYNAMIXEL absolute position 기준: 0~4095, 2048이 중립, 4096 step/rev.
DXL_POSITION_RESOLUTION     = 4096
DXL_CENTER_POSITION         = 2048
DXL_MIN_POSITION            = 0
DXL_MAX_POSITION            = 4095
DXL_DEG_PER_STEP            = 360.0 / DXL_POSITION_RESOLUTION
DXL_STEPS_PER_DEG           = DXL_POSITION_RESOLUTION / 360.0
DXL_MIN_DEG                 = (DXL_MIN_POSITION - DXL_CENTER_POSITION) * DXL_DEG_PER_STEP
DXL_MAX_DEG                 = (DXL_MAX_POSITION - DXL_CENTER_POSITION) * DXL_DEG_PER_STEP

# 0이면 STL을 줄이지 않고 원본 mesh를 렌더링합니다. 중간 샘플링은 표면이 찢어져 보여서 기본은 원본 유지.
MAX_RENDER_TRIANGLES_PER_MESH = 0
STATE_FILENAME = "sdk_gui_state.json"
MIN_TIMELINE_FRAME_MS = 10
DEFAULT_SEQUENCE_FRAME_MS = 100


class SafeProtocol2PacketHandler(Protocol2PacketHandler):
    def _release_port(self, port):
        if hasattr(port, "is_using"):
            port.is_using = False

    def txPacket(self, port, txpacket):
        if getattr(port, "is_using", False):
            self._release_port(port)
        try:
            return super().txPacket(port, txpacket)
        except Exception as exc:
            self._release_port(port)
            print(f"[❌ TX 예외] 다이나믹셀 송신 중 예외 발생: {exc}")
            return COMM_TX_FAIL
        finally:
            self._release_port(port)

    def rxPacket(self, port, fast_option):
        try:
            return super().rxPacket(port, fast_option)
        except Exception as exc:
            self._release_port(port)
            print(f"[❌ RX 예외] 다이나믹셀 수신 중 예외 발생: {exc}")
            return [], COMM_RX_FAIL
        finally:
            self._release_port(port)

    def txRxPacket(self, port, txpacket):
        try:
            return super().txRxPacket(port, txpacket)
        except Exception as exc:
            self._release_port(port)
            print(f"[❌ TX/RX 예외] 다이나믹셀 통신 중 예외 발생: {exc}")
            return None, COMM_TX_FAIL, 0
        finally:
            self._release_port(port)

    def read1ByteTxRx(self, port, dxl_id, address):
        try:
            return super().read1ByteTxRx(port, dxl_id, address)
        except (IndexError, TypeError):
            # SDK가 성공 코드와 함께 길이가 부족한 상태 패킷을 돌려주면
            # 내부에서 data[0] 접근 중 예외가 발생할 수 있습니다.
            self._release_port(port)
            return 0, COMM_RX_FAIL, 0

    def read4ByteTxRx(self, port, dxl_id, address):
        try:
            return super().read4ByteTxRx(port, dxl_id, address)
        except (IndexError, TypeError):
            # SDK의 COMM_SUCCESS 판정과 무관하게 실제 데이터가 4바이트보다
            # 짧은 경우를 정상적인 통신 실패 반환값으로 변환합니다.
            self._release_port(port)
            return 0, COMM_RX_FAIL, 0


class NoWheelSpinBox(QSpinBox):
    """마우스 휠로 값이 우발적으로 바뀌지 않는 숫자 입력칸."""

    def wheelEvent(self, event):
        event.ignore()


class NoWheelSlider(QSlider):
    """마우스 휠은 목록 스크롤에만 사용하고 관절값은 변경하지 않습니다."""

    def wheelEvent(self, event):
        event.ignore()


# ----------------------------------------------------
# 리스트 커스텀 항목 위젯 (1번 탭 라이브러리용)
# ----------------------------------------------------
class FrameItemWidget(QWidget):
    def __init__(self, frame_data, parent_gui):
        super().__init__()
        self.frame_data = frame_data
        self.parent_gui = parent_gui
        # Ubuntu/Qt 테마의 상속 글자색이 흰색이면 흰 목록 배경에서
        # 행이 빈 것처럼 보입니다. 프레임 행의 배경/글자색을 고정합니다.
        self.setStyleSheet(
            "FrameItemWidget { background-color: #ffffff; color: #212121; }"
        )
        
        layout = QHBoxLayout()
        layout.setContentsMargins(5, 5, 5, 5)
        
        self.checkbox = QCheckBox()
        self.checkbox.setVisible(self.parent_gui.is_select_mode)
        self.checkbox.setStyleSheet("spacing: 10px;")
        
        self.label = QLabel(f"[{frame_data['name']}] {frame_data['time_ms']}ms")
        self.label.setStyleSheet("font-size: 13pt; color: #212121; background: transparent;")
        # QLabel이 마우스 클릭을 가로채지 않고 행 위젯이 선택하게 합니다.
        self.label.setAttribute(Qt.WA_TransparentForMouseEvents, True)
        
        self.btn_star = QPushButton('★' if frame_data.get('is_important') else '☆')
        self.btn_star.setFlat(True)
        self.btn_star.setFixedWidth(50)
        self.btn_star.setStyleSheet("font-size: 20pt; color: #FFD700; border: none; background: transparent;")
        self.btn_star.clicked.connect(self.toggle_star)
        
        layout.addWidget(self.checkbox)
        layout.addWidget(self.label)
        layout.addStretch()
        layout.addWidget(self.btn_star)
        
        self.setLayout(layout)

    def set_selected_style(self, selected):
        background = "#90caf9" if selected else "#ffffff"
        self.setStyleSheet(
            f"FrameItemWidget {{ background-color: {background}; color: #212121; }}"
        )

    def select_self_in_library(self):
        """커스텀 행을 누르면 QListWidget의 실제 원본 행을 선택합니다."""
        frame_list = getattr(self.parent_gui, "frame_list_ui1", None)
        if frame_list is None:
            return
        for row in range(frame_list.count()):
            item = frame_list.item(row)
            if frame_list.itemWidget(item) is self:
                frame_list.setCurrentItem(item)
                item.setSelected(True)
                return

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.select_self_in_library()
        super().mousePressEvent(event)
        
    def toggle_star(self):
        self.select_self_in_library()
        is_imp = not self.frame_data.get('is_important', False)
        self.frame_data['is_important'] = is_imp
        self.btn_star.setText('★' if is_imp else '☆')
        self.parent_gui.refresh_library_lists()

# ----------------------------------------------------
# 타임라인 클립 위젯
# ----------------------------------------------------
class TimelineBlockWidget(QFrame):
    def __init__(self, frame_data, seq_idx, parent_gui):
        super().__init__()
        self.frame_data = frame_data
        self.parent_gui = parent_gui
        self.seq_idx = seq_idx

        self.setFrameShape(QFrame.StyledPanel)
        self.set_default_style()
        # 블록의 실제 픽셀 폭이 곧 시간 폭입니다. 고정 60px 최소 폭은 짧은
        # 프레임을 약 160~240ms처럼 보이게 하고 이웃 블록 위에 덮어씌웠습니다.
        self.setMinimumWidth(1)
        
        self.setMouseTracking(True) 
        self.EDGE_MARGIN = 15 
        self.drag_mode = None 

        layout = QVBoxLayout()
        layout.setContentsMargins(2, 2, 2, 2)
        
        top_layout = QHBoxLayout()
        lbl_title = QLabel(f"[{seq_idx+1}] {frame_data['name']}")
        lbl_title.setStyleSheet("font-weight: bold; color: white; border: none; background: transparent; font-size: 11pt;")
        
        btn_del = QPushButton("❌")
        btn_del.setFixedWidth(20)
        btn_del.setStyleSheet("background-color: transparent; border: none; color: #ff5252; font-weight: bold; font-size: 10pt;")
        btn_del.clicked.connect(self.on_delete)
        
        top_layout.addWidget(lbl_title)
        top_layout.addWidget(btn_del)
        
        self.spinbox = QSpinBox()
        # 뒤 프레임을 함께 밀면서 타임라인 자체도 늘릴 수 있어야 하므로 현재
        # max_seq_ms가 아니라 전체 허용 범위(60초)까지 입력을 받습니다.
        max_duration_ms = max(
            MIN_TIMELINE_FRAME_MS,
            60000 - int(frame_data['start_ms']),
        )
        self.spinbox.setRange(MIN_TIMELINE_FRAME_MS, max_duration_ms)
        self.spinbox.setValue(frame_data['time_ms'])
        self.spinbox.setSuffix(" ms")
        self.spinbox.setStyleSheet("font-size: 10pt; border: 1px solid #777; background: #222; color: white; padding: 2px;")
        # 숫자를 입력하는 도중에는 블록 길이를 바꾸지 않고 Enter로 확정합니다.
        self.spinbox.setKeyboardTracking(False)
        self.spinbox.lineEdit().returnPressed.connect(self.apply_spinbox_time)

        self.gap_spinbox = QSpinBox()
        self.gap_spinbox.setRange(0, self.parent_gui.max_seq_ms)
        self.gap_spinbox.setValue(self.parent_gui.frame_gap_ms(frame_data))
        self.gap_spinbox.setSuffix(" ms 앞 프레임 간격")
        self.gap_spinbox.setKeyboardTracking(False)
        self.gap_spinbox.setStyleSheet(
            "font-size: 9pt; border: 1px solid #777; background: #222; color: #ffd54f; padding: 1px;"
        )
        self.gap_spinbox.lineEdit().returnPressed.connect(self.apply_gap_time)
        
        layout.addLayout(top_layout)
        layout.addWidget(self.gap_spinbox)
        layout.addWidget(self.spinbox)
        layout.addStretch() 
        
        self.setLayout(layout)

    def set_default_style(self):
        # 프레임은 더 이상 서로 떨어진 사각 블록으로 그리지 않습니다.
        # 이 위젯은 이름/시간 편집과 드래그 영역만 담당하고, 실제 모션
        # 흐름은 TimelineContainer가 하나의 연결선으로 그립니다.
        self.setStyleSheet(
            "TimelineBlockWidget { background-color: transparent; border: none; }"
        )

    def set_playing_style(self):
        self.setStyleSheet(
            "TimelineBlockWidget { background-color: transparent; border: none; }"
        )

    def calculate_bounds(self):
        seq = sorted(self.parent_gui.motion_sequence, key=lambda x: x['start_ms'])
        idx = next((i for i, frame in enumerate(seq) if frame is self.frame_data), -1)

        self.resize_min_x = 0
        if idx > 0:
            prev_f = seq[idx-1]
            self.resize_min_x = int((prev_f['start_ms'] + prev_f['time_ms']) * self.parent_gui.SCALE)

        # 오른쪽 길이 조절은 다음 프레임에 막히지 않습니다. 놓는 순간 길이
        # 변화량만큼 다음 프레임들을 전부 이동해 기존 간격을 보존합니다.
        self.resize_max_x = int(60000 * self.parent_gui.SCALE)

        self.move_min_x = self.resize_min_x
        self.move_max_x = self.resize_max_x - self.width()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            edge_margin = min(self.EDGE_MARGIN, max(1, self.width() // 3))
            if event.x() < edge_margin: self.drag_mode = 'resize_left'
            elif event.x() >= self.width() - edge_margin: self.drag_mode = 'resize_right'
            else: self.drag_mode = 'move'

            if self.drag_mode == 'move':
                self.frame_data['_drag_original_start_ms'] = self.frame_data['start_ms']
            elif self.drag_mode == 'resize_right':
                self.resize_original_duration_ms = int(self.frame_data['time_ms'])
                
            self.drag_start_global_x = event.globalX()
            self.start_x = self.x()
            self.start_w = self.width()
            self.calculate_bounds()
            self.raise_() 

    def mouseMoveEvent(self, event):
        if not self.drag_mode:
            edge_margin = min(self.EDGE_MARGIN, max(1, self.width() // 3))
            if event.x() < edge_margin or event.x() >= self.width() - edge_margin:
                self.setCursor(Qt.SizeHorCursor) 
            else:
                self.setCursor(Qt.ArrowCursor)   
            return

        delta_x = event.globalX() - self.drag_start_global_x
        
        if self.drag_mode == 'move':
            new_x = self.start_x + delta_x
            max_x = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE) - self.width()
            new_x = max(0, min(new_x, max_x))
            self.move(new_x, self.y())
            self.frame_data['start_ms'] = int(new_x / self.parent_gui.SCALE)
            
        elif self.drag_mode == 'resize_right':
            new_w = self.start_w + delta_x
            max_w = self.resize_max_x - self.start_x
            min_w = max(1, int(math.ceil(MIN_TIMELINE_FRAME_MS * self.parent_gui.SCALE)))
            new_w = max(min_w, min(new_w, max_w))
            self.resize(new_w, self.height())
            
            new_time_ms = int(new_w / self.parent_gui.SCALE)
            self.frame_data['time_ms'] = new_time_ms
            self.spinbox.blockSignals(True)
            self.spinbox.setValue(new_time_ms)
            self.spinbox.blockSignals(False)
            
        elif self.drag_mode == 'resize_left':
            new_x = self.start_x + delta_x
            new_w = self.start_w - delta_x
            
            if new_x < self.resize_min_x:
                diff = self.resize_min_x - new_x
                new_x += diff
                new_w -= diff
            
            min_w = max(1, int(math.ceil(MIN_TIMELINE_FRAME_MS * self.parent_gui.SCALE)))
            if new_w < min_w:
                diff = min_w - new_w
                new_w += diff
                new_x -= diff
                
            self.move(new_x, self.y())
            self.resize(new_w, self.height())
            
            self.frame_data['start_ms'] = int(new_x / self.parent_gui.SCALE)
            self.frame_data['time_ms'] = int(new_w / self.parent_gui.SCALE)
            self.spinbox.blockSignals(True)
            self.spinbox.setValue(self.frame_data['time_ms'])
            self.spinbox.blockSignals(False)

        self.parent_gui.refresh_timeline_meta() 

    def mouseReleaseEvent(self, event):
        if self.drag_mode:
            released_mode = self.drag_mode
            self.drag_mode = None
            self.setCursor(Qt.ArrowCursor)
            if released_mode == 'move':
                self.parent_gui.reorder_motion_frame(self.frame_data, self.x())
            elif released_mode == 'resize_right':
                old_duration_ms = getattr(
                    self,
                    'resize_original_duration_ms',
                    int(self.frame_data['time_ms']),
                )
                new_duration_ms = int(self.frame_data['time_ms'])
                if not self.parent_gui.change_motion_frame_duration(
                    self.frame_data,
                    new_duration_ms,
                    old_duration_ms=old_duration_ms,
                ):
                    self.frame_data['time_ms'] = old_duration_ms
                    QMessageBox.warning(
                        self,
                        "시간 초과",
                        "뒤 프레임까지 이동하면 최대 시퀀스 길이 60000ms를 "
                        "초과하여 원래 시간으로 복구했습니다.",
                    )
            else:
                self.parent_gui.resort_motion_sequence()
            self.parent_gui.refresh_timeline_ui()

    def clamp_time_val(self, val):
        max_allowed_ms = max(
            MIN_TIMELINE_FRAME_MS,
            60000 - int(self.frame_data['start_ms']),
        )
        return max(MIN_TIMELINE_FRAME_MS, min(val, max_allowed_ms))

    def apply_spinbox_time(self):
        self.spinbox.interpretText()
        old_duration_ms = int(self.frame_data['time_ms'])
        val = self.clamp_time_val(self.spinbox.value())
        changed = self.parent_gui.change_motion_frame_duration(
            self.frame_data,
            val,
            old_duration_ms=old_duration_ms,
        )
        if not changed:
            val = old_duration_ms
            QMessageBox.warning(
                self,
                "시간 초과",
                "뒤 프레임까지 이동하면 최대 시퀀스 길이 60000ms를 "
                "초과하여 원래 시간으로 복구했습니다.",
            )
        self.spinbox.blockSignals(True)
        self.spinbox.setValue(val)
        self.spinbox.blockSignals(False)
        self.parent_gui.refresh_timeline_ui()

    def apply_gap_time(self):
        self.gap_spinbox.interpretText()
        requested_gap = self.gap_spinbox.value()
        previous_end = self.parent_gui.previous_frame_end_ms(self.frame_data)
        requested_start = previous_end + requested_gap
        self.frame_data['_drag_original_start_ms'] = self.frame_data['start_ms']
        self.parent_gui.reorder_motion_frame(
            self.frame_data,
            int(requested_start * self.parent_gui.SCALE),
        )
        self.parent_gui.refresh_timeline_ui()

    def on_delete(self):
        self.parent_gui.remove_from_motion_by_idx(self.seq_idx)

# ----------------------------------------------------
# 캔버스 (타임라인 눈금자 및 배경 + 드래그 앤 드롭 수신)
# ----------------------------------------------------
class TimelineContainer(QWidget):
    def __init__(self, parent_gui):
        super().__init__()
        self.parent_gui = parent_gui
        self.show_playhead = False
        self.playhead_x = 0
        self.setAcceptDrops(True) 

    def set_playhead(self, show, x=0):
        self.show_playhead = show
        self.playhead_x = x
        self.update() 

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        
        pen_axis = QPen(QColor(120, 120, 120))
        pen_axis.setWidth(2)
        painter.setPen(pen_axis)
        y_axis = 30 
        
        max_w = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE)
        painter.drawLine(0, y_axis, max_w, y_axis) 
        
        # 확대 배율과 관계없이 시간 글자가 충분히 떨어지도록 간격을 정합니다.
        label_interval_ms = next(
            (
                interval
                for interval in (50, 100, 200, 500, 1000, 2000, 5000, 10000)
                if interval * self.parent_gui.SCALE >= 90
            ),
            10000,
        )
        tick_interval_ms = max(10, label_interval_ms // 5)
        current_ms = 0
        
        while current_ms <= self.parent_gui.max_seq_ms:
            x = int(current_ms * self.parent_gui.SCALE)
            if current_ms % label_interval_ms == 0:
                painter.setPen(QPen(QColor(70, 78, 86), 1, Qt.DashLine))
                painter.drawLine(x, y_axis, x, 172)
                painter.setPen(QPen(QColor(180, 190, 200), 2))
                painter.drawLine(x, y_axis - 9, x, y_axis + 2)
                painter.setPen(QColor("#37474f"))
                painter.drawText(x + 4, y_axis - 6, f"{current_ms} ms")
                painter.setPen(pen_axis)
            elif current_ms % (tick_interval_ms * 2) == 0:
                painter.drawLine(x, y_axis - 6, x, y_axis)
            else:
                painter.drawLine(x, y_axis - 3, x, y_axis)
            current_ms += tick_interval_ms

        # 프레임을 '목표 자세 비아포인트'로 보고, 각 프레임의 실행시간을
        # 점 사이의 선 길이로 표시합니다. 의도적으로 둔 프레임 간 공백은
        # 끊어진 블록 대신 점선으로 이어서 시간 흐름을 한눈에 보이게 합니다.
        path_y = 165
        ordered = sorted(
            self.parent_gui.motion_sequence,
            key=lambda frame: frame['start_ms'],
        )
        previous_end_x = None

        for index, frame in enumerate(ordered):
            start_ms = int(frame['start_ms'])
            duration_ms = int(frame['time_ms'])
            end_ms = start_ms + duration_ms
            start_x = int(start_ms * self.parent_gui.SCALE)
            end_x = int(end_ms * self.parent_gui.SCALE)

            if previous_end_x is not None and start_x > previous_end_x:
                gap_ms = start_ms - previous_end_ms
                gap_pen = QPen(QColor(145, 145, 145))
                gap_pen.setWidth(3)
                gap_pen.setStyle(Qt.DashLine)
                painter.setPen(gap_pen)
                painter.drawLine(previous_end_x, path_y, start_x, path_y)
                gap_center_x = (previous_end_x + start_x) // 2
                painter.setBrush(QColor("#424951"))
                painter.setPen(Qt.NoPen)
                painter.drawRoundedRect(
                    gap_center_x - 40, path_y - 34, 80, 23, 7, 7
                )
                painter.setPen(QColor("#f0f2f4"))
                painter.drawText(
                    gap_center_x - 40,
                    path_y - 34,
                    80,
                    23,
                    Qt.AlignCenter,
                    f"대기 {gap_ms} ms",
                )

            is_current = (
                (self.parent_gui.is_playing or self.parent_gui.is_paused)
                and start_ms <= self.parent_gui.current_timeline_ms <= end_ms
            )
            segment_color = (
                QColor(105, 240, 174) if is_current else QColor(66, 165, 245)
            )
            segment_pen = QPen(segment_color)
            segment_pen.setWidth(8)
            segment_pen.setCapStyle(Qt.RoundCap)
            painter.setPen(segment_pen)
            painter.drawLine(start_x, path_y, end_x, path_y)

            painter.setPen(QPen(QColor(235, 245, 255), 2))
            painter.setBrush(segment_color)
            if index == 0:
                painter.drawEllipse(start_x - 6, path_y - 6, 12, 12)
                painter.setPen(QColor("#455a64"))
                painter.drawText(
                    start_x - 45,
                    path_y + 12,
                    90,
                    17,
                    Qt.AlignCenter,
                    f"시작 {start_ms} ms",
                )
            painter.drawEllipse(end_x - 7, path_y - 7, 14, 14)

            segment_width = max(1, end_x - start_x)
            segment_center_x = (start_x + end_x) // 2
            duration_badge_width = max(68, min(108, segment_width - 8))
            painter.setBrush(
                QColor("#1b5e20") if is_current else QColor("#0d47a1")
            )
            painter.setPen(QPen(segment_color, 1))
            painter.drawRoundedRect(
                segment_center_x - duration_badge_width // 2,
                path_y - 35,
                duration_badge_width,
                24,
                8,
                8,
            )
            painter.setPen(QColor("#ffffff"))
            painter.drawText(
                segment_center_x - duration_badge_width // 2,
                path_y - 35,
                duration_badge_width,
                24,
                Qt.AlignCenter,
                f"↔ {duration_ms} ms",
            )

            # 프레임 이름은 해당 구간의 가운데에 두고, 종료 시각만 기존처럼
            # 비아포인트 끝에 둡니다.
            label_y = path_y + 12 + (index % 2) * 32
            painter.setPen(QColor("#263238"))
            painter.drawText(
                segment_center_x - 70,
                label_y,
                140,
                16,
                Qt.AlignCenter,
                str(frame.get("name", f"Frame {index + 1}")),
            )
            painter.setPen(QColor("#e65100"))
            painter.drawText(
                end_x - 70,
                label_y + 16,
                140,
                17,
                Qt.AlignCenter,
                f"{end_ms} ms",
            )
            previous_end_x = end_x
            previous_end_ms = end_ms

        if self.show_playhead:
            pen_playhead = QPen(QColor(255, 152, 0)) 
            pen_playhead.setWidth(3)
            painter.setPen(pen_playhead)
            painter.drawLine(self.playhead_x, 0, self.playhead_x, self.height())
            current_ms = int(self.playhead_x / max(0.001, self.parent_gui.SCALE))
            painter.setBrush(QColor("#e65100"))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(
                self.playhead_x - 45, self.height() - 25, 90, 23, 7, 7
            )
            painter.setPen(QColor("#ffffff"))
            painter.drawText(
                self.playhead_x - 45,
                self.height() - 25,
                90,
                23,
                Qt.AlignCenter,
                f"현재 {current_ms} ms",
            )

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.parent_gui.is_playing:
                self.parent_gui.pause_motion_sequence()
            self.scrub(event.x())

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.scrub(event.x())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            # 마지막 목표 자세도 완만한 로봇 추종기에 전달합니다.
            self.scrub(event.x())

    def scrub(self, x, force_robot=False):
        max_w = int(self.parent_gui.max_seq_ms * self.parent_gui.SCALE)
        x = max(0, min(x, max_w))
        self.set_playhead(True, x)
        t_ms = int(x / self.parent_gui.SCALE)
        self.parent_gui.scrub_timeline(t_ms, force_robot=force_robot)

    def dragEnterEvent(self, event):
        event.accept()


class JointTrajectoryWidget(QWidget):
    """선택한 관절 하나를 처음부터 끝까지 끊김 없는 궤적으로 표시합니다."""

    COLORS = [
        QColor("#42a5f5"), QColor("#ef5350"), QColor("#66bb6a"),
        QColor("#ffca28"), QColor("#ab47bc"), QColor("#26c6da"),
        QColor("#ff7043"), QColor("#8d6e63"), QColor("#ec407a"),
        QColor("#7e57c2"), QColor("#9ccc65"), QColor("#78909c"),
    ]

    def __init__(self, parent_gui):
        super().__init__()
        self.parent_gui = parent_gui
        self.setMinimumHeight(305)
        self.setMouseTracking(True)
        self.setStyleSheet("background-color: #161a1f; border: 1px solid #555;")
        self.setToolTip(
            "가로축: 시퀀스 시간 / 세로축: 관절 목표각\n"
            "클릭하거나 드래그하면 같은 시점의 3D 자세를 확인합니다."
        )

    def graph_rect(self):
        return self.rect().adjusted(58, 62, -18, -42)

    def selected_joint_ids(self):
        joint_id = self.parent_gui.combo_trajectory_group.currentData()
        try:
            return [int(joint_id)]
        except (TypeError, ValueError):
            return []

    def time_to_x(self, t_ms):
        graph = self.graph_rect()
        total_ms = max(1, self.parent_gui.motion_end_ms())
        return graph.left() + graph.width() * max(0, min(t_ms, total_ms)) / total_ms

    def x_to_time(self, x):
        graph = self.graph_rect()
        ratio = (x - graph.left()) / max(1, graph.width())
        ratio = max(0.0, min(1.0, ratio))
        return int(round(ratio * max(1, self.parent_gui.motion_end_ms())))

    def trajectory_samples(self, joint_ids):
        end_ms = self.parent_gui.motion_end_ms()
        if end_ms <= 0 or not joint_ids:
            return []
        # 화면 픽셀보다 과도하게 많이 계산하지 않되 10ms 이하의 프레임
        # 경계도 누락하지 않도록 모든 키프레임 시작/끝 시각을 함께 넣습니다.
        sample_count = max(80, min(320, self.graph_rect().width() // 3))
        times = {
            int(round(end_ms * index / sample_count))
            for index in range(sample_count + 1)
        }
        for frame in self.parent_gui.motion_sequence:
            start_ms = int(frame.get("start_ms", 0))
            end_frame_ms = start_ms + int(frame.get("time_ms", 0))
            times.update((start_ms, end_frame_ms))

        samples = []
        for t_ms in sorted(times):
            angles, _, _ = self.parent_gui.timeline_state_at(t_ms)
            samples.append(
                (t_ms, {j_id: float(angles.get(j_id, 0.0)) for j_id in joint_ids})
            )
        return samples

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing, True)
        graph = self.graph_rect()
        joint_ids = self.selected_joint_ids()
        samples = self.trajectory_samples(joint_ids)

        painter.fillRect(self.rect(), QColor("#161a1f"))
        painter.fillRect(graph, QColor("#20262d"))
        selected_name = self.parent_gui.combo_trajectory_group.currentText()
        painter.setPen(QColor("#dce6ef"))
        painter.drawText(
            12, 24,
            f"단일 연속 궤적: {selected_name} (클릭하면 그 시점의 전체 3D 자세 확인)",
        )

        if not samples:
            painter.setPen(QColor("#aab4bd"))
            painter.drawText(graph, Qt.AlignCenter, "시퀀스에 프레임을 추가하세요.")
            return

        values = [
            angle
            for _, angles in samples
            for angle in angles.values()
        ]
        min_angle = min(values)
        max_angle = max(values)
        if max_angle - min_angle < 10.0:
            center = (min_angle + max_angle) / 2.0
            min_angle, max_angle = center - 5.0, center + 5.0
        padding = max(3.0, (max_angle - min_angle) * 0.12)
        min_angle -= padding
        max_angle += padding

        def angle_to_y(angle):
            ratio = (float(angle) - min_angle) / max(1e-9, max_angle - min_angle)
            return graph.bottom() - ratio * graph.height()

        # 각도/시간 격자
        painter.setPen(QPen(QColor("#38414a"), 1, Qt.DashLine))
        for index in range(5):
            y = graph.top() + graph.height() * index / 4.0
            angle = max_angle - (max_angle - min_angle) * index / 4.0
            painter.drawLine(graph.left(), int(y), graph.right(), int(y))
            painter.setPen(QColor("#aab4bd"))
            painter.drawText(3, int(y) - 8, 50, 16, Qt.AlignRight, f"{angle:.1f}°")
            painter.setPen(QPen(QColor("#38414a"), 1, Qt.DashLine))

        end_ms = self.parent_gui.motion_end_ms()
        for index in range(6):
            t_ms = int(round(end_ms * index / 5.0))
            x = int(self.time_to_x(t_ms))
            painter.drawLine(x, graph.top(), x, graph.bottom())
            painter.setPen(QColor("#aab4bd"))
            painter.drawText(x - 35, graph.bottom() + 7, 70, 18, Qt.AlignCenter, f"{t_ms}ms")
            painter.setPen(QPen(QColor("#38414a"), 1, Qt.DashLine))

        # 프레임 도착 지점: 그래프 전체를 가르는 비아포인트 선
        for index, frame in enumerate(
            sorted(self.parent_gui.motion_sequence, key=lambda item: item["start_ms"])
        ):
            arrival_ms = int(frame["start_ms"] + frame["time_ms"])
            x = int(self.time_to_x(arrival_ms))
            painter.setPen(QPen(QColor("#71808e"), 1, Qt.DotLine))
            painter.drawLine(x, graph.top(), x, graph.bottom())
            painter.setPen(QColor("#d5dde5"))
            label_y = graph.top() - 20 if index % 2 == 0 else graph.top() - 5
            painter.drawText(
                x - 55, label_y, 110, 16, Qt.AlignCenter,
                str(frame.get("name", f"Frame {index + 1}")),
            )

        # 선택한 관절 하나를 첫 시점부터 마지막 시점까지 단 한 번의
        # QPainterPath로 그려 프레임 경계에서도 선이 끊기지 않게 합니다.
        name_by_id = {
            int(item["id"]): str(item["name"])
            for item in self.parent_gui.joint_data
        }
        for color_index, j_id in enumerate(joint_ids):
            color = QColor("#42a5f5")
            path = QPainterPath()
            for sample_index, (t_ms, angles) in enumerate(samples):
                point_x = self.time_to_x(t_ms)
                point_y = angle_to_y(angles[j_id])
                if sample_index == 0:
                    path.moveTo(point_x, point_y)
                else:
                    path.lineTo(point_x, point_y)
            painter.setPen(QPen(color, 4))
            painter.drawPath(path)

            # 저장 프레임은 같은 선 위의 점일 뿐 별도 선분으로 나누지 않습니다.
            sample_by_time = {t_ms: angles for t_ms, angles in samples}
            painter.setBrush(QColor("#ffffff"))
            painter.setPen(QPen(color, 3))
            for frame in self.parent_gui.motion_sequence:
                arrival_ms = int(frame["start_ms"] + frame["time_ms"])
                arrival_angles = sample_by_time.get(arrival_ms)
                if arrival_angles is None:
                    arrival_angles, _, _ = self.parent_gui.timeline_state_at(arrival_ms)
                point_x = self.time_to_x(arrival_ms)
                point_y = angle_to_y(float(arrival_angles.get(j_id, 0.0)))
                painter.drawEllipse(int(point_x) - 6, int(point_y) - 6, 12, 12)

            legend_x = graph.left()
            legend_y = self.height() - 24
            painter.setPen(QPen(color, 3))
            painter.drawLine(legend_x, legend_y + 7, legend_x + 16, legend_y + 7)
            painter.setPen(QColor("#dce6ef"))
            painter.drawText(
                legend_x + 21, legend_y, 300, 15,
                Qt.AlignLeft, f"{j_id}: {name_by_id.get(j_id, j_id)}",
            )

        # 현재 확인 중인 시점과 그때의 선택 관절 각도
        playhead_x = int(self.time_to_x(self.parent_gui.current_timeline_ms))
        painter.setPen(QPen(QColor("#ff9800"), 3))
        painter.drawLine(playhead_x, graph.top(), playhead_x, graph.bottom())
        painter.setBrush(QColor("#ff9800"))
        current_angles, _, _ = self.parent_gui.timeline_state_at(
            self.parent_gui.current_timeline_ms
        )
        if joint_ids:
            current_angle = float(current_angles.get(joint_ids[0], 0.0))
            current_y = int(angle_to_y(current_angle))
            painter.drawEllipse(playhead_x - 7, current_y - 7, 14, 14)
            painter.setPen(QColor("#ffffff"))
            painter.drawText(
                playhead_x - 48,
                max(graph.top(), current_y - 27),
                96,
                20,
                Qt.AlignCenter,
                f"{current_angle:.1f}°",
            )

    def scrub(self, x):
        if not self.parent_gui.motion_sequence:
            return
        if self.parent_gui.is_playing:
            self.parent_gui.pause_motion_sequence()
        t_ms = self.x_to_time(x)
        timeline_x = int(t_ms * self.parent_gui.SCALE)
        self.parent_gui.timeline_container.set_playhead(True, timeline_x)
        self.parent_gui.timeline_scroll.ensureVisible(
            timeline_x,
            self.parent_gui.timeline_scroll.height() // 2,
            50,
            0,
        )
        self.parent_gui.scrub_timeline(t_ms, update_visuals=True)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.scrub(event.x())

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.scrub(event.x())


class SequenceTimelineBlockWidget(QFrame):
    """3페이지에서 저장 시퀀스 하나를 나타내는 타임라인 블록."""

    def __init__(self, entry, index, parent_gui):
        super().__init__()
        self.entry = entry
        self.index = index
        self.parent_gui = parent_gui
        self.drag_start_global_x = 0
        self.start_x = 0

        self.setStyleSheet(
            "SequenceTimelineBlockWidget { background-color: #5e35b1; "
            "border: 2px solid #9575cd; border-radius: 6px; }"
        )
        layout = QVBoxLayout(self)
        layout.setContentsMargins(6, 4, 6, 4)
        top = QHBoxLayout()
        label = QLabel(f"[{index + 1}] {entry['name']}")
        label.setStyleSheet("color: white; font-weight: bold; font-size: 11pt;")
        delete_button = QPushButton("❌")
        delete_button.setFixedWidth(24)
        delete_button.setStyleSheet("background: transparent; border: none; color: #ff8a80;")
        delete_button.clicked.connect(lambda: self.parent_gui.remove_sequence_composer_entry(self.index))
        top.addWidget(label)
        top.addStretch()
        top.addWidget(delete_button)
        leading_wait_ms = int(entry.get('leading_wait_ms', 0))
        wait_text = (
            f"맨 앞 대기 {leading_wait_ms}ms / "
            if leading_wait_ms > 0
            else "맨 앞 대기 없음 / "
        )
        duration = QLabel(
            f"{wait_text}전체 {entry['time_ms']}ms / "
            f"{entry['frame_count']} 프레임"
        )
        duration.setStyleSheet("color: white;")
        layout.addLayout(top)
        layout.addWidget(duration)

        # 시퀀스 박스 안에 대기 구간과 모든 프레임을 시간 비율로
        # 나란히 표시해, 원본 시퀀스를 불러오지 않고도 내용을
        # 바로 확인할 수 있게 합니다.
        segment_layout = QHBoxLayout()
        segment_layout.setSpacing(2)
        segment_layout.setContentsMargins(0, 2, 0, 0)
        cursor_ms = 0
        for frame in entry.get("frames", []):
            start_ms = max(0, int(frame.get("start_ms", 0)))
            frame_ms = max(1, int(frame.get("time_ms", 0)))
            if start_ms > cursor_ms:
                gap_ms = start_ms - cursor_ms
                gap_label = QLabel(f"대기\n{gap_ms}ms")
                gap_label.setAlignment(Qt.AlignCenter)
                gap_label.setStyleSheet(
                    "background:#455a64; color:#fff59d; border:1px dashed #b0bec5; "
                    "border-radius:4px; font-size:9pt;"
                )
                gap_label.setToolTip(
                    f"{cursor_ms}ms → {start_ms}ms / 직전 자세 유지"
                )
                gap_label.setAttribute(Qt.WA_TransparentForMouseEvents, True)
                segment_layout.addWidget(gap_label, gap_ms)

            end_ms = start_ms + frame_ms
            frame_label = QLabel(
                f"{frame.get('name', 'Frame')}\n"
                f"{start_ms}→{end_ms}ms\n{frame_ms}ms"
            )
            frame_label.setAlignment(Qt.AlignCenter)
            frame_label.setStyleSheet(
                "background:#1e88e5; color:white; border:1px solid #90caf9; "
                "border-radius:4px; font-size:9pt; font-weight:bold;"
            )
            frame_label.setToolTip(
                f"{frame.get('name', 'Frame')} / 시작 {start_ms}ms / "
                f"도착 {end_ms}ms / 이동 {frame_ms}ms"
            )
            frame_label.setAttribute(Qt.WA_TransparentForMouseEvents, True)
            segment_layout.addWidget(frame_label, frame_ms)
            cursor_ms = max(cursor_ms, end_ms)
        layout.addLayout(segment_layout)

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.drag_start_global_x = event.globalX()
            self.start_x = self.x()
            self.raise_()

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            new_x = self.start_x + event.globalX() - self.drag_start_global_x
            max_x = max(0, self.parent_gui.composer_timeline_width() - self.width())
            self.move(max(0, min(new_x, max_x)), self.y())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.parent_gui.reorder_sequence_composer_entry(self.entry, self.x())


class SequenceTimelineContainer(QWidget):
    def __init__(self, parent_gui):
        super().__init__()
        self.parent_gui = parent_gui
        self.show_playhead = False
        self.playhead_x = 0
        self.setAcceptDrops(True)

    def set_playhead(self, show, x=0):
        self.show_playhead = show
        self.playhead_x = x
        self.update()

    def paintEvent(self, event):
        super().paintEvent(event)
        painter = QPainter(self)
        axis_pen = QPen(QColor(120, 120, 120))
        axis_pen.setWidth(2)
        painter.setPen(axis_pen)
        painter.drawLine(0, 30, self.parent_gui.composer_timeline_width(), 30)
        duration = self.parent_gui.composer_total_duration()
        tick = 100
        current = 0
        while current <= duration:
            x = int(current * self.parent_gui.COMPOSER_SCALE)
            if current % 500 == 0:
                painter.drawLine(x, 20, x, 30)
                painter.drawText(x + 3, 16, f"{current}ms")
            else:
                painter.drawLine(x, 26, x, 30)
            current += tick
        if self.show_playhead:
            playhead_pen = QPen(QColor(255, 152, 0))
            playhead_pen.setWidth(3)
            painter.setPen(playhead_pen)
            painter.drawLine(self.playhead_x, 0, self.playhead_x, self.height())

    def dragEnterEvent(self, event):
        event.accept()

    def dragMoveEvent(self, event):
        event.accept()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            if self.parent_gui.is_playing:
                self.parent_gui.pause_motion_sequence()
            self.scrub(event.x())

    def mouseMoveEvent(self, event):
        if event.buttons() & Qt.LeftButton:
            self.scrub(event.x())

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.scrub(event.x())

    def scrub(self, x):
        if not self.parent_gui.sequence_composer_entries:
            return
        if self.parent_gui.playback_context != "composer":
            if not self.parent_gui.activate_composer_playback():
                return
        max_x = int(self.parent_gui.composer_total_duration() * self.parent_gui.COMPOSER_SCALE)
        x = max(0, min(x, max_x))
        self.set_playhead(True, x)
        self.parent_gui.scrub_timeline(int(x / self.parent_gui.COMPOSER_SCALE))

    def dropEvent(self, event):
        item = self.parent_gui.composer_source_list.currentItem()
        if item is None:
            return
        self.parent_gui.add_sequence_to_composer(
            sequence_idx=item.data(Qt.UserRole),
            drop_x=event.pos().x(),
        )
        event.accept()

    def dragMoveEvent(self, event):
        event.accept()

    def dropEvent(self, event):
        selected_items = self.parent_gui.frame_list_all.selectedItems()
        if not selected_items: return

        drop_x = event.pos().x()
        start_ms = int(drop_x / self.parent_gui.SCALE)

        for item in selected_items:
            original_idx = item.data(Qt.UserRole)
            new_frame = copy.deepcopy(self.parent_gui.frames[original_idx])
            new_frame["source_frame_id"] = new_frame.get("frame_id")
            # 이 기능 추가 이후 새로 구성하는 타임라인에만 조기 도착을
            # 적용합니다. 기존 저장 시퀀스에는 이 필드가 없어 영향이 없습니다.
            new_frame["lift_early_arrival"] = True
            # 라이브러리의 시간은 단일 자세 적용용이다. 시퀀스 블록은
            # 독립적인 기본 시간으로 시작하고 타임라인에서 따로 편집한다.
            new_frame['time_ms'] = DEFAULT_SEQUENCE_FRAME_MS
            
            new_frame['start_ms'] = start_ms
            self.parent_gui.motion_sequence.append(new_frame)
            placed = self.parent_gui.reorder_motion_frame(
                new_frame, int(start_ms * self.parent_gui.SCALE)
            )
            if not placed:
                QMessageBox.warning(self, "공간 부족", "겹치지 않고 배치할 빈 공간이 없습니다.")
                break
            start_ms = new_frame['start_ms'] + new_frame['time_ms']

        self.parent_gui.refresh_timeline_ui()
        self.parent_gui.frame_list_all.clearSelection()
        event.accept()


# ----------------------------------------------------
# 내장 URDF/STL OpenGL 뷰어 (urdfpy/pyqtgraph 없이 동작)
# ----------------------------------------------------
def _parse_vec3(text, default=(0.0, 0.0, 0.0)):
    if not text:
        return default
    vals = [float(v) for v in text.split()]
    if len(vals) != 3:
        return default
    return tuple(vals)


def _transform_from_xyz_rpy(xyz, rpy):
    import numpy as np
    x, y, z = xyz
    roll, pitch, yaw = rpy

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=float)
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=float)

    mat = np.eye(4, dtype=float)
    mat[:3, :3] = rz @ ry @ rx
    mat[:3, 3] = [x, y, z]
    return mat


def _axis_angle_transform(axis, angle):
    import numpy as np
    axis = np.array(axis, dtype=float)
    norm = np.linalg.norm(axis)
    if norm == 0:
        return np.eye(4, dtype=float)

    x, y, z = axis / norm
    c = math.cos(angle)
    s = math.sin(angle)
    t = 1.0 - c

    mat = np.eye(4, dtype=float)
    mat[:3, :3] = np.array([
        [t*x*x + c,     t*x*y - s*z,   t*x*z + s*y],
        [t*x*y + s*z,   t*y*y + c,     t*y*z - s*x],
        [t*x*z - s*y,   t*y*z + s*x,   t*z*z + c],
    ], dtype=float)
    return mat


def _load_stl_triangles(path, max_triangles=MAX_RENDER_TRIANGLES_PER_MESH):
    triangles = []
    with open(path, "rb") as f:
        data = f.read()

    is_binary = False
    if len(data) >= 84:
        tri_count = struct.unpack_from("<I", data, 80)[0]
        is_binary = 84 + tri_count * 50 == len(data)

    if is_binary:
        stride = max(1, math.ceil(tri_count / max_triangles)) if max_triangles else 1
        for tri_idx in range(0, tri_count, stride):
            if max_triangles and len(triangles) >= max_triangles:
                break
            offset = 84 + tri_idx * 50
            vals = struct.unpack_from("<12fH", data, offset)
            normal = vals[0:3]
            v1 = vals[3:6]
            v2 = vals[6:9]
            v3 = vals[9:12]
            triangles.append((normal, v1, v2, v3))
        return triangles

    text = data.decode("utf-8", errors="ignore").splitlines()
    normal = (0.0, 0.0, 1.0)
    vertices = []
    for line in text:
        parts = line.strip().split()
        if not parts:
            continue
        if parts[0] == "facet" and len(parts) >= 5 and parts[1] == "normal":
            normal = (float(parts[2]), float(parts[3]), float(parts[4]))
        elif parts[0] == "vertex" and len(parts) >= 4:
            vertices.append((float(parts[1]), float(parts[2]), float(parts[3])))
            if len(vertices) == 3:
                triangles.append((normal, vertices[0], vertices[1], vertices[2]))
                vertices = []
    if max_triangles and len(triangles) > max_triangles:
        stride = max(1, math.ceil(len(triangles) / max_triangles))
        triangles = triangles[::stride][:max_triangles]
    return triangles


class SimpleURDFModel:
    def __init__(self):
        self.links = set()
        self.root_link = None
        self.joints = {}
        self.children = {}
        self.visuals = []
        self.mesh_cache = {}
        self.triangle_count = 0
        self.bounds_min = None
        self.bounds_max = None
        self.end_effector_links = []

    @classmethod
    def load(cls, urdf_path, load_visuals=True):
        import numpy as np

        model = cls()
        base_dir = os.path.dirname(os.path.abspath(urdf_path))
        root = ET.parse(urdf_path).getroot()

        for link in root.findall("link"):
            link_name = link.get("name")
            model.links.add(link_name)
            if not load_visuals:
                continue
            for visual in link.findall("visual"):
                origin = visual.find("origin")
                xyz = _parse_vec3(origin.get("xyz") if origin is not None else None)
                rpy = _parse_vec3(origin.get("rpy") if origin is not None else None)

                mesh = visual.find("./geometry/mesh")
                if mesh is None:
                    continue
                mesh_filename = mesh.get("filename", "")
                mesh_filename = mesh_filename.replace("package://step/", "")
                mesh_path = mesh_filename if os.path.isabs(mesh_filename) else os.path.join(base_dir, mesh_filename)
                if not os.path.exists(mesh_path):
                    print(f"[URDF 경고] Mesh 파일 없음: {mesh_path}")
                    continue

                material_color = (0.85, 0.88, 0.92, 1.0)
                color = visual.find("./material/color")
                if color is not None and color.get("rgba"):
                    vals = [float(v) for v in color.get("rgba").split()]
                    if len(vals) == 4:
                        material_color = (vals[0], vals[1], vals[2], 1.0)

                if mesh_path not in model.mesh_cache:
                    model.mesh_cache[mesh_path] = _load_stl_triangles(mesh_path)

                visual_entry = {
                    "link": link_name,
                    "origin": _transform_from_xyz_rpy(xyz, rpy),
                    "mesh": mesh_path,
                    "color": material_color,
                }
                model.visuals.append(visual_entry)

        child_links = set()
        for joint in root.findall("joint"):
            joint_name = joint.get("name")
            parent = joint.find("parent")
            child = joint.find("child")
            if parent is None or child is None:
                continue

            origin = joint.find("origin")
            xyz = _parse_vec3(origin.get("xyz") if origin is not None else None)
            rpy = _parse_vec3(origin.get("rpy") if origin is not None else None)
            axis_node = joint.find("axis")
            axis = _parse_vec3(axis_node.get("xyz") if axis_node is not None else None, (0.0, 0.0, 1.0))

            parent_link = parent.get("link")
            child_link = child.get("link")
            child_links.add(child_link)
            joint_data = {
                "name": joint_name,
                "type": joint.get("type", "fixed"),
                "parent": parent_link,
                "child": child_link,
                "origin": _transform_from_xyz_rpy(xyz, rpy),
                "axis": axis,
            }
            model.joints[joint_name] = joint_data
            model.children.setdefault(parent_link, []).append(joint_data)

        root_candidates = sorted(model.links - child_links)
        model.root_link = root_candidates[0] if root_candidates else "base_link"
        model.end_effector_links = sorted(child_links - set(model.children.keys()))
        model.triangle_count = sum(len(triangles) for triangles in model.mesh_cache.values())
        model._compute_bounds()
        return model

    def _compute_link_poses(self, joint_angles):
        import numpy as np

        poses = {self.root_link: np.eye(4, dtype=float)}
        stack = [self.root_link]
        while stack:
            parent_link = stack.pop()
            parent_pose = poses[parent_link]
            for joint in self.children.get(parent_link, []):
                angle = joint_angles.get(joint["name"], 0.0)
                if joint["type"] in ("revolute", "continuous"):
                    motion = _axis_angle_transform(joint["axis"], angle)
                else:
                    motion = np.eye(4, dtype=float)
                poses[joint["child"]] = parent_pose @ joint["origin"] @ motion
                stack.append(joint["child"])
        return poses

    def _compute_bounds(self):
        import numpy as np

        poses = self._compute_link_poses({})
        mins = []
        maxs = []
        for visual in self.visuals:
            pose = poses.get(visual["link"], np.eye(4, dtype=float)) @ visual["origin"]
            triangles = self.mesh_cache.get(visual["mesh"], [])
            if not triangles:
                continue
            pts = np.array([vertex for _, v1, v2, v3 in triangles for vertex in (v1, v2, v3)], dtype=float)
            pts_h = np.c_[pts, np.ones(len(pts))]
            world = (pose @ pts_h.T).T[:, :3]
            mins.append(world.min(axis=0))
            maxs.append(world.max(axis=0))

        if mins:
            self.bounds_min = np.vstack(mins).min(axis=0)
            self.bounds_max = np.vstack(maxs).max(axis=0)
        else:
            self.bounds_min = np.array([-0.2, -0.2, -0.2], dtype=float)
            self.bounds_max = np.array([0.2, 0.2, 0.2], dtype=float)


class URDFGLViewer(QOpenGLWidget):
    def __init__(self, robot_model, parent=None):
        super().__init__(parent)
        self.robot_model = robot_model
        self.joint_angles = {}
        self.setMinimumWidth(350)
        self.setMinimumHeight(300)
        # URDF는 Z-up인데 OpenGL 카메라는 기본적으로 -Z 방향을 봅니다.
        # X축 회전을 음수로 두어 로봇의 +Z가 화면 위쪽으로 오게 맞춥니다.
        self.rot_x = -65.0
        self.rot_z = 25.0
        self.distance = 2.2
        self.last_mouse_pos = None
        self.mesh_display_lists = {}
        self.pending_joint_angles = {}
        self.update_pending = False

        import numpy as np
        center = (self.robot_model.bounds_min + self.robot_model.bounds_max) / 2.0
        size = self.robot_model.bounds_max - self.robot_model.bounds_min
        max_size = max(float(size.max()), 0.001)
        self.model_center = center
        self.model_scale = 1.45 / max_size

    def set_joint_angles(self, joint_angles):
        self.pending_joint_angles = dict(joint_angles)
        if self.update_pending:
            return
        self.update_pending = True
        QTimer.singleShot(33, self.flush_joint_update)

    def flush_joint_update(self):
        self.joint_angles = self.pending_joint_angles
        self.update_pending = False
        self.update()

    def initializeGL(self):
        from OpenGL.GL import (
            glBegin, glBlendFunc, glClearColor, glColorMaterial, glEnable, glEnd,
            glEndList, glGenLists, glLightfv, glNewList, glNormal3f,
            glShadeModel, GL_AMBIENT, GL_AMBIENT_AND_DIFFUSE, GL_BLEND,
            GL_COLOR_MATERIAL, GL_DEPTH_TEST, GL_DIFFUSE, GL_FRONT_AND_BACK,
            GL_LIGHT0, GL_LIGHTING, GL_NORMALIZE, GL_ONE_MINUS_SRC_ALPHA,
            GL_POSITION, GL_SMOOTH, GL_SRC_ALPHA, GL_COMPILE, GL_TRIANGLES,
            glVertex3f
        )

        glClearColor(0.13, 0.13, 0.13, 1.0)
        glEnable(GL_DEPTH_TEST)
        glEnable(GL_LIGHTING)
        glEnable(GL_LIGHT0)
        glEnable(GL_COLOR_MATERIAL)
        glEnable(GL_NORMALIZE)
        glEnable(GL_BLEND)
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA)
        glShadeModel(GL_SMOOTH)
        glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
        glLightfv(GL_LIGHT0, GL_POSITION, [1.2, -1.6, 2.4, 0.0])
        glLightfv(GL_LIGHT0, GL_AMBIENT, [0.35, 0.35, 0.35, 1.0])
        glLightfv(GL_LIGHT0, GL_DIFFUSE, [0.82, 0.82, 0.82, 1.0])

        self.mesh_display_lists.clear()
        for mesh_path, triangles in self.robot_model.mesh_cache.items():
            list_id = glGenLists(1)
            glNewList(list_id, GL_COMPILE)
            glBegin(GL_TRIANGLES)
            for normal, v1, v2, v3 in triangles:
                glNormal3f(float(normal[0]), float(normal[1]), float(normal[2]))
                glVertex3f(float(v1[0]), float(v1[1]), float(v1[2]))
                glVertex3f(float(v2[0]), float(v2[1]), float(v2[2]))
                glVertex3f(float(v3[0]), float(v3[1]), float(v3[2]))
            glEnd()
            glEndList()
            self.mesh_display_lists[mesh_path] = list_id

    def resizeGL(self, width, height):
        from OpenGL.GL import glMatrixMode, glLoadIdentity, glViewport, GL_MODELVIEW, GL_PROJECTION
        from OpenGL.GLU import gluPerspective

        height = max(1, height)
        glViewport(0, 0, width, height)
        glMatrixMode(GL_PROJECTION)
        glLoadIdentity()
        gluPerspective(45.0, width / float(height), 0.01, 20.0)
        glMatrixMode(GL_MODELVIEW)

    def paintGL(self):
        import numpy as np
        from OpenGL.GL import (
            glCallList, glClear, glColor4f, glLoadIdentity, glMatrixMode,
            glMultMatrixf, glPopMatrix, glPushMatrix, glRotatef, glScalef,
            glTranslatef, GL_COLOR_BUFFER_BIT, GL_DEPTH_BUFFER_BIT, GL_MODELVIEW
        )

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glMatrixMode(GL_MODELVIEW)
        glLoadIdentity()
        glTranslatef(0.0, 0.0, -self.distance)
        glRotatef(self.rot_x, 1.0, 0.0, 0.0)
        glRotatef(self.rot_z, 0.0, 0.0, 1.0)
        glScalef(self.model_scale, self.model_scale, self.model_scale)
        glTranslatef(-float(self.model_center[0]), -float(self.model_center[1]), -float(self.model_center[2]))

        link_poses = self.robot_model._compute_link_poses(self.joint_angles)
        for visual in self.robot_model.visuals:
            pose = link_poses.get(visual["link"])
            if pose is None:
                continue
            transform = pose @ visual["origin"]
            color = visual["color"]
            list_id = self.mesh_display_lists.get(visual["mesh"])
            if list_id is None:
                continue

            glPushMatrix()
            glMultMatrixf(transform.T.astype(np.float32).flatten())
            glColor4f(float(color[0]), float(color[1]), float(color[2]), float(color[3]))
            glCallList(list_id)
            glPopMatrix()

    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if self.last_mouse_pos is None:
            return
        dx = event.x() - self.last_mouse_pos.x()
        dy = event.y() - self.last_mouse_pos.y()
        self.rot_z += dx * 0.5
        self.rot_x += dy * 0.5
        self.last_mouse_pos = event.pos()
        self.update()

    def mouseReleaseEvent(self, event):
        self.last_mouse_pos = None

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta > 0:
            self.distance *= 0.9
        else:
            self.distance *= 1.1
        self.distance = max(0.3, min(8.0, self.distance))
        self.update()


# ----------------------------------------------------
# 메인 에디터 클래스
# ----------------------------------------------------
class SDKMotionEditor(QWidget):
    def __init__(self):
        super().__init__()
        # 50ms 눈금을 읽기 쉽게 하고 20ms 프레임도 약 50px 폭으로
        # 편집할 수 있도록 기본 타임라인 확대를 250%로 시작합니다.
        self.SCALE = 2.5
        self.max_seq_ms = 5000 
        
        self.frames = []          
        self.motion_sequence = [] 
        self.saved_sequences = [] 
        self.loaded_sequence_id = None
        self.is_select_mode = False 
        self.show_favorite_frames_only = False
        # 전체 목록과 즐겨찾기는 서로 다른 화면이므로 QListWidget의 숨은
        # currentRow를 실행 대상처럼 공유하지 않습니다. frame_id를 기준으로
        # 실제 편집/실행 대상과 마지막 전체 목록 선택을 따로 보관합니다.
        self.selected_frame_id = None
        self.last_full_frame_id = None
        self.default_pose_frame_id = None
        self.autosave_enabled = False
        self.state_file_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), STATE_FILENAME)
        self.state_backup_dir = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), ".sdk_gui_backups"
        )
        self.undo_state_history = []
        
        self.anim_timer = QTimer(self)
        self.anim_timer.setTimerType(Qt.PreciseTimer)
        self.anim_timer.setInterval(CONTROL_INTERVAL_MS)
        self.anim_timer.timeout.connect(self.anim_step)
        self.live_angle_timer = QTimer(self)
        self.live_angle_timer.setInterval(100)
        self.live_angle_timer.timeout.connect(self.refresh_live_joint_angles)
        self.live_angle_read_failures = 0
        self.robot_scrub_timer = QTimer(self)
        self.robot_scrub_timer.setTimerType(Qt.PreciseTimer)
        self.robot_scrub_timer.setInterval(CONTROL_INTERVAL_MS)
        self.robot_scrub_timer.timeout.connect(self.step_robot_scrub)
        self.robot_scrub_target_angles = {}
        self.robot_scrub_command_angles = {}
        self.frame_apply_timer = QTimer(self)
        self.frame_apply_timer.setTimerType(Qt.PreciseTimer)
        self.frame_apply_timer.setInterval(CONTROL_INTERVAL_MS)
        self.frame_apply_timer.timeout.connect(self.step_frame_apply)
        self.frame_apply_start_time = 0.0
        self.frame_apply_duration = 0.5
        self.frame_apply_start_angles = {}
        self.frame_apply_target_angles = {}
        self.frame_apply_ids = []
        self.frame_apply_name = ""
        self.frame_apply_frame = None
        self.frame_apply_on_complete = None
        self.is_playing = False
        self.is_paused = False
        self.playback_paused_by_button = False
        self.playback_context = "motion"
        self.composer_motion_backup = None
        self.playback_repeat_target = 1
        self.playback_repeat_current = 1
        self.playback_speed = 1.0
        self.current_timeline_ms = 0
        self.current_seq_idx = 0
        self.anim_start_time = 0
        self.anim_duration = 0
        self.start_angles = {}
        self.last_playback_ui_time = 0.0
        self.time_based_profile_ids = set()
        self.active_timed_frame_token = None
        self.last_timed_feedback_time = 0.0
        self.timed_profile_error_reported = False
        self.timed_gate_frame = None
        self.timed_gate_wait_started = 0.0
        self.timed_gate_last_check = 0.0
        
        self.execute_on_real_robot = False
        self.robot_sync_enabled = False
        
        self.joint_data = [
            {"id": 0, "name": "Head_Pan", "type": "28"}, {"id": 1, "name": "Head_Tilt", "type": "28"},
            {"id": 2, "name": "R_Shoulder_Pitch", "type": "64"}, {"id": 3, "name": "L_Shoulder_Pitch", "type": "64"},
            {"id": 4, "name": "R_Shoulder_Roll", "type": "28"}, {"id": 5, "name": "L_Shoulder_Roll", "type": "28"},
            {"id": 6, "name": "R_Elbow_Pitch", "type": "28"}, {"id": 7, "name": "L_Elbow_Pitch", "type": "28"},
            {"id": 8, "name": "R_Wrist_Yaw", "type": "28"}, {"id": 9, "name": "L_Wrist_Yaw", "type": "28"},
            {"id": 10, "name": "R_Hip_Yaw", "type": "106"}, {"id": 11, "name": "Waist_Yaw", "type": "106"},      
            {"id": 12, "name": "L_Hip_Yaw", "type": "106"}, {"id": 15, "name": "R_Hip_Roll", "type": "106"},
            {"id": 16, "name": "L_Hip_Roll", "type": "106"}, {"id": 13, "name": "R_Hip_Pitch", "type": "106"},
            {"id": 14, "name": "L_Hip_Pitch", "type": "106"}, {"id": 17, "name": "R_Knee_Pitch", "type": "106"},
            {"id": 18, "name": "L_Knee_Pitch", "type": "106"}, {"id": 19, "name": "R_Ankle_Pitch", "type": "106"},
            {"id": 20, "name": "L_Ankle_Pitch", "type": "106"}, {"id": 21, "name": "R_Ankle_Roll", "type": "106"},
            {"id": 22, "name": "L_Ankle_Roll", "type": "106"}
        ]
        
        self.urdf_joint_map = {
            0: "Neck_yaw", 1: "Neck_pitch", 2: "R_Arm_shoulder_yaw", 3: "L_Arm_shoulder_yaw",
            4: "R_Arm_pitch", 5: "L_Arm_pitch", 6: "R_Arm_elbow", 7: "L_Arm_elbow",
            8: "R_arm_hand", 9: "L_Arm_hand", 10: "R_Leg_hip_yaw", 11: "Waist",
            12: "L_Leg_hip_yaw", 15: "R_Leg_hip_roll", 16: "L_Leg_hip_roll",
            13: "R_Leg_hip_pitch", 14: "L_Leg_hip_pitch", 17: "R_Leg_knee",
            18: "L_Leg_knee", 19: "R_Leg_ankle_pitch", 20: "L_Leg_ankle_pitch",
            21: "R_Leg_ankle_roll", 22: "L_Leg_ankle_roll"
        }
        
        # 좌우반전은 하체 관절만 적용합니다. 각 쌍은 좌우 값을 교환하면서
        # 부호를 반전하고, 머리/팔/허리 등 여기에 없는 관절은 그대로 둡니다.
        self.mirror_map = {
            10: (12, -1),
            15: (16, -1),
            21: (22, -1),
            13: (14, -1),
            17: (18, -1),
            19: (20, -1),
        }
        
        self.joints = {joint["id"]: 0 for joint in self.joint_data}
        self.feedback_angles = self.joints.copy()
        self.commanded_angles = self.joints.copy()
        self.commanded_joint_ids = set()
        self.fk_feedback_sample_count = 0
        self.fk_feedback_read_failures = 0
        self.last_fk_error_log_time = 0.0
        self.last_fk_read_error_log_time = 0.0
        self.editing_loaded_pose = False
        self.sliders = {}
        self.spinboxes = {}
        self.torque_btns = {}
        self.torque_group_btns = {}
        self.torque_groups = {
            "오른팔": [2, 4, 6, 8],
            "왼팔": [3, 5, 7, 9],
            "오른쪽 다리": [10, 13, 15, 17, 19, 21],
            "왼쪽 다리": [12, 14, 16, 18, 20, 22],
        }
        self.trajectory_joint_groups = {
            "왼쪽 다리": [12, 14, 16, 18, 20, 22],
            "오른쪽 다리": [10, 13, 15, 17, 19, 21],
            "양쪽 다리": [10, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22],
            "상체·팔": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11],
            "전체 관절": [int(item["id"]) for item in self.joint_data],
        }
        self.online_joints = []
        
        self.load_3d_robot_urdf()
        
        # 🚀 UI를 구성하기 전에, 사용 가능한 포트를 안전하게 먼저 확보합니다.
        print("\n================ [로봇 하드웨어 통신 디버그 로그] ================")
        actual_port = self.find_dynamixel_port()
        
        self.portHandler = PortHandler(actual_port)
        self.packetHandler = SafeProtocol2PacketHandler()
        
        # Read/Write 동기화 객체 추가
        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)
        self.groupSyncRead = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        self.groupSyncWriteTorque = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_TORQUE_ENABLE, 1)
        self.groupSyncWriteDriveMode = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            ADDR_DRIVE_MODE,
            1,
        )
        self.groupSyncWriteProfileAcceleration = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            ADDR_PROFILE_ACCELERATION,
            LEN_PROFILE_VALUE,
        )
        self.groupSyncWriteProfileVelocity = GroupSyncWrite(
            self.portHandler,
            self.packetHandler,
            ADDR_PROFILE_VELOCITY,
            LEN_PROFILE_VALUE,
        )

        self.port_opened = False
        if self.portHandler.openPort():
            if self.portHandler.setBaudRate(BAUDRATE):
                self.port_opened = True
                print(f"[✅ 시스템 오픈 완료] 최종 연결 포트: {actual_port}, 속도: {BAUDRATE} bps")
            else:
                print(f"[❌ 보레이트 설정 실패]")
        else:
            print(f"[❌ 통신 포트 오픈 실패]")
            
        self.initUI()
        self.load_persistent_state()
        self.autosave_enabled = True
        
        # 🤖 안전 제일: 포트 연결 성공 시 현재 로봇 포즈부터 스캔하고, 토크는 "OFF" 상태(티칭 모드)로 기동합니다!
        if self.port_opened:
            self.lbl_conn_status.setText(f"🟢 하드웨어 연결 성공 ({actual_port} | {BAUDRATE}bps)")
            self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #1e4620; color: #69f0ae; padding: 8px; border-radius: 5px; margin-bottom: 5px;")
            
            # 🚀 현재 실제로 전선에 응답하는 관절 ID만 발라냅니다.
            self.detect_online_joints()
            
            # 🚀 생존한 관절 각도만 SyncRead로 안전하게 긁어와 GUI 및 3D 모델에 매핑시킵니다.
            self.sync_initial_angles()

            # 🚨 로봇이 튀지 않도록 완전히 릴랙스된 티칭(토크 OFF) 상태로 프로그램을 켭니다.
            self.set_all_torque_off()

            # 토크 상태와 무관하게 Present Position을 계속 읽어 프레임 제작
            # 화면의 각도값을 실제 관절 위치로 유지합니다.
            self.live_angle_timer.start()
        else:
            self.lbl_conn_status.setText(f"🔴 연결 실패 (포트/권한 확인)")
            self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #5c1d1d; color: #ff5252; padding: 8px; border-radius: 5px; margin-bottom: 5px;")
        print("==================================================================\n")

    # 🚀 [완벽 수정 1] 23개 관절 중 진짜 살아있는 모터 ID를 수색합니다 (오프라인 모터 배제용)
    def detect_online_joints(self):
        self.online_joints = []
        if not self.port_opened: return
        print("📡 [온라인 모터 감지] 23개 관절 중 실제 작동 중인 모터를 수색합니다...")
        for j_id in range(23):
            # 1. 핑 테스트로 확인
            _, dxl_comm_result, _ = self.packetHandler.ping(self.portHandler, j_id)
            if dxl_comm_result == COMM_SUCCESS:
                self.online_joints.append(j_id)
                print(f"   🟢 ID {j_id:02d} - 온라인 확인!")
            else:
                # 2. 백업으로 1바이트 읽기 시도 (간혹 status return level 설정으로 인해 ping을 무시하는 경우 대비)
                _, dxl_comm_result2, _ = self.packetHandler.read1ByteTxRx(self.portHandler, j_id, ADDR_TORQUE_ENABLE)
                if dxl_comm_result2 == COMM_SUCCESS:
                    self.online_joints.append(j_id)
                    print(f"   🟢 ID {j_id:02d} - 온라인 확인! (Read1Byte 백업 성공)")
                else:
                    print(f"   🔴 ID {j_id:02d} - 오프라인 (미응답)")
        print(f"📊 [감지 완료] 총 {len(self.online_joints)}/23 관절이 통신 가능 상태입니다.")

    @staticmethod
    def dxl_u32_param(value):
        value = max(0, min(0xFFFFFFFF, int(value)))
        return [
            DXL_LOBYTE(DXL_LOWORD(value)),
            DXL_HIBYTE(DXL_LOWORD(value)),
            DXL_LOBYTE(DXL_HIWORD(value)),
            DXL_HIBYTE(DXL_HIWORD(value)),
        ]

    def read_1byte_register(self, j_id, address):
        """초기 설정용 1바이트 레지스터를 패킷 오류 시 재시도해 읽습니다."""
        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.005)
                self.portHandler.clearPort()
            value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler,
                int(j_id),
                int(address),
            )
            if dxl_comm_result == COMM_SUCCESS:
                if dxl_error != 0:
                    print(
                        f"[⚠️ 설정 레지스터 읽기 중 모터 경고] ID {j_id}: "
                        f"{self.packetHandler.getRxPacketError(dxl_error)}"
                    )
                return int(value)
        return None

    def configure_unlimited_motor_profiles(self, target_ids=None):
        """MX Protocol 2.0의 내부 속도/가속도 프로파일 제한을 해제합니다."""
        if not self.port_opened:
            return False

        ids = sorted(set(self.online_joints if target_ids is None else target_ids))
        if not ids:
            return False

        zero_profile = self.dxl_u32_param(0)
        profile_writers = (
            ("가속도", self.groupSyncWriteProfileAcceleration),
            ("속도", self.groupSyncWriteProfileVelocity),
        )
        for profile_name, writer in profile_writers:
            writer.clearParam()
            added_count = 0
            for j_id in ids:
                if writer.addParam(int(j_id), zero_profile):
                    added_count += 1
                else:
                    print(f"[❌ 프로파일 설정 준비 실패] ID {j_id} {profile_name}")

            if added_count != len(ids):
                return False

            dxl_comm_result = writer.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(
                    f"[❌ 프로파일 설정 실패] {profile_name}: "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )
                return False

        print(
            f"[✅ 모터 프로파일 초기화] ID {ids} Profile Acceleration/Velocity = 0"
        )
        return True

    def configure_time_based_drive_mode(self, target_ids=None):
        """토크 OFF 상태의 MX를 프레임 도착시간 기반 프로파일로 설정합니다."""
        if not self.port_opened:
            return False

        ids = sorted(set(self.online_joints if target_ids is None else target_ids))
        if not ids:
            return False
        if set(ids).issubset(self.time_based_profile_ids):
            return self.configure_unlimited_motor_profiles(ids)

        drive_modes = {}
        unsupported_ids = []
        read_failed_ids = []
        for j_id in ids:
            firmware = self.read_1byte_register(j_id, ADDR_FIRMWARE_VERSION)
            drive_mode = self.read_1byte_register(j_id, ADDR_DRIVE_MODE)
            if firmware is None or drive_mode is None:
                read_failed_ids.append(j_id)
                continue
            if firmware < MIN_TIME_PROFILE_FIRMWARE:
                unsupported_ids.append((j_id, firmware))
                continue
            drive_modes[j_id] = drive_mode

        if read_failed_ids:
            print(f"[❌ 시간 프로파일 확인 실패] 레지스터 읽기 실패 ID {read_failed_ids}")
            return False
        if unsupported_ids:
            print(
                "[❌ 시간 프로파일 미지원] 펌웨어 V42 이상 필요: "
                + ", ".join(f"ID {j_id}=V{version}" for j_id, version in unsupported_ids)
            )
            return False

        change_ids = [
            j_id for j_id, drive_mode in drive_modes.items()
            if not (drive_mode & DRIVE_MODE_TIME_BASED_BIT)
        ]
        if change_ids:
            # Drive Mode(10)은 EEPROM이므로 이 함수는 토크 OFF 이후에만 호출합니다.
            self.groupSyncWriteDriveMode.clearParam()
            for j_id in change_ids:
                new_mode = drive_modes[j_id] | DRIVE_MODE_TIME_BASED_BIT
                if not self.groupSyncWriteDriveMode.addParam(j_id, [new_mode]):
                    print(f"[❌ 시간 프로파일 설정 준비 실패] ID {j_id}")
                    return False
            dxl_comm_result = self.groupSyncWriteDriveMode.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(
                    "[❌ 시간 프로파일 Drive Mode 설정 실패] "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )
                return False
            # EEPROM 기록이 완료된 뒤 실제 bit가 들어갔는지 검증합니다.
            time.sleep(0.02)

        verify_failed_ids = []
        for j_id in ids:
            drive_mode = self.read_1byte_register(j_id, ADDR_DRIVE_MODE)
            if drive_mode is None or not (drive_mode & DRIVE_MODE_TIME_BASED_BIT):
                verify_failed_ids.append(j_id)
        if verify_failed_ids:
            print(f"[❌ 시간 프로파일 검증 실패] ID {verify_failed_ids}")
            return False

        if not self.configure_unlimited_motor_profiles(ids):
            return False

        self.time_based_profile_ids.update(ids)
        print(f"[✅ 시간 기반 프로파일 준비] ID {ids} Drive Mode bit2=1")
        return True

    def set_time_profile_duration(self, target_ids, duration_ms, accel_ms=0):
        """동일 프레임의 도착시간과 시작/종료 가감속 시간을 지정합니다.

        accel_ms가 0이면 기존 동작과 동일합니다. Time-based Profile에서
        Profile Acceleration은 시작 가속과 종료 감속 구간에 함께 적용됩니다.
        """
        ids = sorted(set(int(j_id) for j_id in target_ids))
        if not ids or not set(ids).issubset(self.time_based_profile_ids):
            return False

        duration_ms = max(1, min(MAX_TIME_PROFILE_MS, int(math.ceil(duration_ms))))
        accel_ms = max(0, min(duration_ms // 2, int(math.ceil(accel_ms))))

        # 일반 프레임에도 0을 명시적으로 전송해 직전 [착지] 프레임의
        # 가감속 설정이 다음 프레임에 남지 않도록 합니다.
        acceleration_param = self.dxl_u32_param(accel_ms)
        self.groupSyncWriteProfileAcceleration.clearParam()
        for j_id in ids:
            if not self.groupSyncWriteProfileAcceleration.addParam(
                j_id, acceleration_param
            ):
                print(f"[❌ 가감속 설정 준비 실패] ID {j_id}")
                return False
        dxl_comm_result = self.groupSyncWriteProfileAcceleration.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(
                f"[❌ 가감속 설정 실패] {accel_ms}ms: "
                f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
            )
            return False

        duration_param = self.dxl_u32_param(duration_ms)
        self.groupSyncWriteProfileVelocity.clearParam()
        for j_id in ids:
            if not self.groupSyncWriteProfileVelocity.addParam(j_id, duration_param):
                print(f"[❌ 도착시간 설정 준비 실패] ID {j_id}")
                return False
        dxl_comm_result = self.groupSyncWriteProfileVelocity.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(
                f"[❌ 도착시간 설정 실패] {duration_ms}ms: "
                f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
            )
            return False
        return True

    # 🚀 [완벽 수정 2] 부팅 시 감지된 관절의 실시간 각도를 일괄 매핑시킵니다.
    def sync_initial_angles(self):
        if not self.online_joints: 
            print("   [⚠️ 경고] 생존한 모터가 전혀 감지되지 않아 부팅 초기 각도 로드를 스킵합니다.")
            return
            
        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)
            
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result == COMM_SUCCESS:
            print("   [✅ 초기화 스캔] 생존한 물리 관절 각도를 대시보드에 완벽 연동했습니다.")
            for j_id in self.online_joints:
                if self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    dxl_present_position = self.groupSyncRead.getData(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    angle_deg = self.dxl_position_to_angle(dxl_present_position)
                    self.update_joint_display(j_id, angle_deg, update_robot=False)
                    self.feedback_angles[j_id] = angle_deg
                    self.commanded_angles[j_id] = angle_deg
                    self.commanded_joint_ids.add(j_id)
            self.update_3d_robot()
        else:
            print(f"   [⚠️ 경고] 기동 시 로봇 각도 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

    def refresh_live_joint_angles(self):
        """토크 ON/OFF와 무관하게 온라인 관절의 실제 각도를 갱신합니다."""
        # 모션/프레임 제어 중에는 전용 실기 피드백 경로가 읽고 있으므로,
        # 100ms 라이브 타이머의 중복 읽기만 건너뜁니다.
        if (not self.port_opened or not self.online_joints or self.is_playing
                or self.frame_apply_timer.isActive()):
            return

        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.live_angle_read_failures += 1
            # 일시적인 패킷 충돌은 다음 100ms 주기에 복구되므로 로그 폭주를
            # 피하고 연속 실패할 때만 상태를 알립니다.
            if self.live_angle_read_failures == 10:
                print(
                    "[⚠️ 실시간 각도 갱신] 10회 연속 읽기 실패: "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )
            return

        self.live_angle_read_failures = 0
        for j_id in self.online_joints:
            if not self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                continue
            dxl_present_position = self.groupSyncRead.getData(
                j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            angle_deg = self.dxl_position_to_angle(dxl_present_position)
            self.feedback_angles[j_id] = angle_deg
            # 저장 자세를 불러와 편집 중일 때는 실기 피드백이 입력값을
            # 100ms마다 덮어쓰지 않게 하고, 실제각은 feedback에만 보관합니다.
            if not self.editing_loaded_pose:
                self.update_joint_display(j_id, angle_deg, update_robot=False)

    def set_joint_connection_ui(self, j_id, is_online, torque_on=False):
        if j_id not in self.torque_btns:
            return

        btn = self.torque_btns[j_id]
        btn.blockSignals(True)
        btn.setEnabled(is_online)
        btn.setChecked(torque_on if is_online else False)
        btn.setText("ON" if is_online and torque_on else "OFF")

        if not is_online:
            btn.setStyleSheet("background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px;")
        elif torque_on:
            btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px;")
        else:
            btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")

        # 각도 편집은 토크 상태와 분리합니다. 토크 OFF에서는 GUI/프레임 값만
        # 바뀌고, 실제 Goal Position 전송은 sync_values에서 계속 차단됩니다.
        if j_id in self.sliders:
            self.sliders[j_id].setEnabled(is_online)
        if j_id in self.spinboxes:
            self.spinboxes[j_id].setEnabled(is_online)

        btn.blockSignals(False)

    def rescan_online_motors(self):
        print("🔄 [수동 재탐색] 온라인 모터 목록을 다시 스캔합니다...")
        if not self.port_opened:
            QMessageBox.warning(self, "통신 에러", "포트가 열려있지 않아 모터를 재탐색할 수 없습니다.")
            return

        previous_online = set(self.online_joints)
        self.detect_online_joints()
        online_set = set(self.online_joints)
        read_success_ids = []
        torque_on_ids = []

        for j_id in range(23):
            if j_id not in online_set:
                self.set_joint_connection_ui(j_id, False, False)
                continue

            angle_deg = self.read_present_angle(j_id)
            if angle_deg is not None:
                self.update_joint_display(j_id, angle_deg, update_robot=False)
                read_success_ids.append(j_id)

            torque_on = self.read_torque_enabled(j_id)
            if torque_on is None:
                torque_on = self.torque_btns[j_id].isChecked()
            if torque_on:
                torque_on_ids.append(j_id)

            self.set_joint_connection_ui(j_id, True, torque_on)

        if read_success_ids:
            self.update_3d_robot()

        newly_online = sorted(online_set - previous_online)
        lost_ids = sorted(previous_online - online_set)
        self.lbl_conn_status.setText(f"🔄 재탐색 완료: 온라인 {len(online_set)}/23개, 각도 갱신 {len(read_success_ids)}개")
        self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #1e4620; color: #69f0ae; padding: 8px; border-radius: 5px; margin-bottom: 5px;" if online_set else "font-weight: bold; font-size: 14pt; background-color: #5c1d1d; color: #ff5252; padding: 8px; border-radius: 5px; margin-bottom: 5px;")

        result_msg = (
            f"온라인 모터: {len(online_set)}/23개\n"
            f"각도 갱신: {len(read_success_ids)}개\n"
            f"토크 ON 감지: {len(torque_on_ids)}개"
        )
        if newly_online:
            result_msg += f"\n새로 연결됨: {newly_online}"
        if lost_ids:
            result_msg += f"\n끊김 감지: {lost_ids}"

        if online_set:
            QMessageBox.information(self, "재탐색 완료", result_msg)
        else:
            QMessageBox.warning(self, "재탐색 완료", result_msg)

    # 🚀 [완벽 수정 3] 실제로 하드웨어에 존재하는 포트만 탐색하여 Fallback 시 에러 및 크래시가 발생하는 것을 원천 차단합니다!
    def find_dynamixel_port(self):
        print("🔍 [자동 포트 탐색] 진짜 U2D2(다이나믹셀)가 연결된 포트를 스캔합니다...")
        possible_ports = ['/dev/ttyUSB0', '/dev/ttyUSB1', '/dev/ttyUSB2', '/dev/ttyUSB3']
        existing_ports = [p for p in possible_ports if os.path.exists(p)]
        
        if not existing_ports:
            print("  [❌ 경고] 시스템에 물리적으로 연결된 USB 시리얼 장치(/dev/ttyUSB*)가 전혀 존재하지 않습니다!")
            return DEVICENAME # 기본 포트 반환
            
        for test_port in existing_ports:
            print(f"  👉 테스트 중: {test_port}")
            temp_port = PortHandler(test_port)
            temp_packet = SafeProtocol2PacketHandler()
            
            if temp_port.openPort():
                if temp_port.setBaudRate(BAUDRATE):
                    # 핑 테스트 (0~22번 전 관절 ID 영역 중 단 하나라도 응답하면 유효한 U2D2 버스로 간주)
                    for test_id in range(23):
                        _, dxl_comm_result, _ = temp_packet.ping(temp_port, test_id)
                        if dxl_comm_result == COMM_SUCCESS:
                            print(f"  [🎉 탐색 성공] ID {test_id}번 모터가 응답했습니다! 진짜 U2D2 포트는 {test_port} 입니다.")
                            temp_port.closePort()
                            return test_port
                temp_port.closePort()
                
        # 포트 탐색 및 핑 스캔은 실패했지만, 물리적으로 존재하는 유효한 포트 중 첫 번째 포트를 안전하게 기본 연결 통로로 반환합니다.
        fallback_port = existing_ports[0]
        print(f"  [⚠️ 탐색 실패] 응답하는 모터가 없으나, 시스템에 실존하는 최선의 포트인 {fallback_port}로 크래시 방지용 가상 연결을 수립합니다.")
        return fallback_port

    def closeEvent(self, event):
        self.save_persistent_state()
        if hasattr(self, 'live_angle_timer'):
            self.live_angle_timer.stop()
        if hasattr(self, 'portHandler') and self.portHandler.is_open:
            self.portHandler.closePort()
            print("[✅ 포트 닫힘] 프로그램을 종료하여 포트를 안전하게 반납했습니다.")
        event.accept()

    def normalize_frame_data(self, frame):
        normalized = copy.deepcopy(frame)
        normalized["name"] = str(normalized.get("name", f"Frame {len(self.frames) + 1}"))
        normalized["time_ms"] = int(normalized.get("time_ms", 500))
        normalized["angles"] = self.normalize_angles(normalized.get("angles", {}))
        normalized["torques"] = {int(j): bool(v) for j, v in normalized.get("torques", {}).items()}
        normalized["is_important"] = bool(normalized.get("is_important", False))
        if "start_ms" in normalized:
            normalized["start_ms"] = int(normalized.get("start_ms", 0))
        return normalized

    def repair_legacy_sequence_timing(self, frames):
        """이전 버전에서 라이브러리 시간이 복사되어 생긴 겹침을 보정한다."""
        ordered = sorted(frames, key=lambda frame: frame.get("start_ms", 0))
        repaired = 0
        for current, following in zip(ordered, ordered[1:]):
            current_start = int(current.get("start_ms", 0))
            next_start = int(following.get("start_ms", 0))
            current_end = current_start + int(
                current.get("time_ms", DEFAULT_SEQUENCE_FRAME_MS)
            )
            if current_end <= next_start:
                continue
            available_ms = next_start - current_start
            if available_ms >= MIN_TIMELINE_FRAME_MS:
                current["time_ms"] = available_ms
            else:
                current["time_ms"] = MIN_TIMELINE_FRAME_MS
                following["start_ms"] = current_start + MIN_TIMELINE_FRAME_MS
            repaired += 1
        return repaired

    def rebuild_frame_list_ui(self):
        if not hasattr(self, 'frame_list_ui1'):
            return
        restore_id = self.last_full_frame_id
        self.frame_list_ui1.clear()
        restored_row = -1
        for row, frame_data in enumerate(self.frames):
            item = QListWidgetItem(self.frame_list_ui1)
            custom_widget = FrameItemWidget(frame_data, self)
            item.setSizeHint(custom_widget.sizeHint())
            self.frame_list_ui1.setItemWidget(item, custom_widget)
            if frame_data.get("frame_id") == restore_id:
                restored_row = row
        if restored_row >= 0:
            self.frame_list_ui1.setCurrentRow(restored_row)
            self.frame_list_ui1.scrollToItem(self.frame_list_ui1.item(restored_row))
        self.refresh_library_lists()

    def save_persistent_state(self):
        if not getattr(self, 'autosave_enabled', False):
            return
        try:
            state = self.current_persistent_state_data()
            previous_state = None
            if os.path.exists(self.state_file_path):
                try:
                    with open(self.state_file_path, 'r', encoding='utf-8') as previous_file:
                        previous_state = json.load(previous_file)
                except (OSError, ValueError, TypeError):
                    previous_state = None

            # 내용이 실제로 바뀔 때만 이전 상태를 메모리와 디스크에 남깁니다.
            # 같은 UI 갱신에서 save가 여러 번 호출돼도 중복 백업되지 않습니다.
            if previous_state is not None and previous_state != state:
                self.undo_state_history.append(copy.deepcopy(previous_state))
                self.undo_state_history = self.undo_state_history[-50:]
                os.makedirs(self.state_backup_dir, exist_ok=True)
                backup_path = os.path.join(
                    self.state_backup_dir,
                    f"state_{time.time_ns()}.json",
                )
                with open(backup_path, 'w', encoding='utf-8') as backup_file:
                    json.dump(previous_state, backup_file, indent=2, ensure_ascii=False)
                backup_files = sorted(
                    name for name in os.listdir(self.state_backup_dir)
                    if name.startswith("state_") and name.endswith(".json")
                )
                for old_name in backup_files[:-50]:
                    try:
                        os.remove(os.path.join(self.state_backup_dir, old_name))
                    except OSError:
                        pass
            with open(self.state_file_path, 'w', encoding='utf-8') as f:
                json.dump(state, f, indent=2, ensure_ascii=False)
        except Exception as e:
            print(f"[⚠️ 자동 저장 실패] {type(e).__name__}: {e}")

    def current_persistent_state_data(self):
        return {
            "version": 1,
            "frames": copy.deepcopy(self.frames),
            "saved_sequences": copy.deepcopy(self.saved_sequences),
            "motion_sequence": copy.deepcopy(self.motion_sequence),
            "max_seq_ms": self.max_seq_ms,
            "loaded_sequence_id": self.loaded_sequence_id,
            "default_pose_frame_id": self.default_pose_frame_id,
            "timeline_zoom_percent": int(round(self.SCALE * 100)),
        }

    @staticmethod
    def describe_state_restore(current_state, restored_state, limit=20):
        """사용자가 알아볼 수 있게 복구 전후 차이를 요약합니다."""
        changes = []
        current_frames = current_state.get("frames", [])
        restored_frames = restored_state.get("frames", [])

        def frame_key(frame):
            return frame.get("frame_id") or ("name", frame.get("name"))

        current_by_key = {frame_key(frame): frame for frame in current_frames}
        restored_by_key = {frame_key(frame): frame for frame in restored_frames}
        for key in current_by_key.keys() - restored_by_key.keys():
            changes.append(f"프레임 제거 복구: '{current_by_key[key].get('name', 'Frame')}'이(가) 없어짐")
        for key in restored_by_key.keys() - current_by_key.keys():
            changes.append(f"프레임 삭제 취소: '{restored_by_key[key].get('name', 'Frame')}' 복원")
        for key in current_by_key.keys() & restored_by_key.keys():
            before = current_by_key[key]
            after = restored_by_key[key]
            frame_name = after.get("name", before.get("name", "Frame"))
            before_angles = {int(j): float(v) for j, v in before.get("angles", {}).items()}
            after_angles = {int(j): float(v) for j, v in after.get("angles", {}).items()}
            for joint_id in sorted(before_angles.keys() | after_angles.keys()):
                old_angle = before_angles.get(joint_id)
                new_angle = after_angles.get(joint_id)
                if old_angle is None or new_angle is None or abs(old_angle - new_angle) > 1e-9:
                    old_text = "없음" if old_angle is None else f"{old_angle:.2f}°"
                    new_text = "없음" if new_angle is None else f"{new_angle:.2f}°"
                    changes.append(
                        f"[{frame_name}] 모터 {joint_id}: {old_text} → {new_text}"
                    )
            old_time = int(before.get("time_ms", 0))
            new_time = int(after.get("time_ms", 0))
            if old_time != new_time:
                changes.append(f"[{frame_name}] 시간: {old_time}ms → {new_time}ms")

        current_sequences = {
            str(sequence.get("name", "")) for sequence in current_state.get("saved_sequences", [])
        }
        restored_sequences = {
            str(sequence.get("name", "")) for sequence in restored_state.get("saved_sequences", [])
        }
        for name in sorted(restored_sequences - current_sequences):
            changes.append(f"시퀀스 삭제 취소: '{name}' 복원")
        for name in sorted(current_sequences - restored_sequences):
            changes.append(f"시퀀스 추가 취소: '{name}' 제거")

        if current_state.get("motion_sequence") != restored_state.get("motion_sequence"):
            changes.append("현재 타임라인 구성이 이전 상태로 복구됨")
        if current_state.get("max_seq_ms") != restored_state.get("max_seq_ms"):
            changes.append(
                f"타임라인 길이: {current_state.get('max_seq_ms')}ms → "
                f"{restored_state.get('max_seq_ms')}ms"
            )
        if not changes:
            changes.append("저장 상태의 내부 데이터가 이전 값으로 복구됨")
        omitted = max(0, len(changes) - limit)
        visible = changes[:limit]
        if omitted:
            visible.append(f"그 외 {omitted}개 변경")
        return "\n".join(visible)

    def apply_persistent_state_data(self, state):
        """백업 JSON의 프레임/시퀀스/타임라인을 현재 GUI에 적용합니다."""
        self.frames = [self.normalize_frame_data(frame) for frame in state.get("frames", [])]
        for frame in self.frames:
            frame.setdefault("frame_id", uuid.uuid4().hex)
        self.saved_sequences = []
        for seq in state.get("saved_sequences", []):
            sequence = {
                "sequence_id": str(seq.get("sequence_id") or uuid.uuid4().hex),
                "name": str(seq.get("name", "Sequence")),
                "max_seq_ms": int(seq.get("max_seq_ms", 5000)),
                "repeat_count": max(1, int(seq.get("repeat_count", 1))),
                "playback_speed": max(0.1, min(5.0, float(seq.get("playback_speed", 1.0)))),
                "repeatable": bool(seq.get("repeatable", True)),
                "completion": copy.deepcopy(seq.get("completion", {
                    "position_tolerance_deg": 2.0,
                    "settle_duration_ms": 80,
                    "settle_timeout_ms": 3000,
                })),
                "frames": [self.normalize_frame_data(frame) for frame in seq.get("frames", [])],
            }
            self.repair_legacy_sequence_timing(sequence["frames"])
            self.saved_sequences.append(sequence)
        self.motion_sequence = [
            self.normalize_frame_data(frame) for frame in state.get("motion_sequence", [])
        ]
        self.repair_legacy_sequence_timing(self.motion_sequence)
        self.link_sequence_frames_to_library()
        self.max_seq_ms = int(state.get("max_seq_ms", self.max_seq_ms))
        self.loaded_sequence_id = state.get("loaded_sequence_id")
        self.default_pose_frame_id = state.get("default_pose_frame_id")
        zoom_percent = max(25, min(400, int(state.get("timeline_zoom_percent", 250))))
        self.SCALE = zoom_percent / 100.0
        if hasattr(self, "spin_timeline_zoom"):
            self.spin_timeline_zoom.blockSignals(True)
            self.spin_timeline_zoom.setValue(zoom_percent)
            self.spin_timeline_zoom.blockSignals(False)
        if hasattr(self, "spin_max_time"):
            self.spin_max_time.setValue(self.max_seq_ms)
        self.rebuild_frame_list_ui()
        self.refresh_sequence_list()
        self.refresh_timeline_ui()
        self.refresh_loaded_sequence_indicator()

    def undo_last_gui_change(self):
        """직전 자동 저장 상태를 복구합니다. 재시작 후에도 디스크 백업을 사용합니다."""
        if self.is_playing:
            self.stop_motion_sequence()
        backup_files = []
        if os.path.isdir(self.state_backup_dir):
            backup_files = sorted(
                name for name in os.listdir(self.state_backup_dir)
                if name.startswith("state_") and name.endswith(".json")
            )
        state = self.undo_state_history.pop() if self.undo_state_history else None
        if state is not None and backup_files:
            try:
                os.remove(os.path.join(self.state_backup_dir, backup_files[-1]))
                backup_files.pop()
            except OSError:
                pass
        if state is None:
            if backup_files:
                backup_path = os.path.join(self.state_backup_dir, backup_files[-1])
                try:
                    with open(backup_path, 'r', encoding='utf-8') as backup_file:
                        state = json.load(backup_file)
                    os.remove(backup_path)
                except (OSError, ValueError, TypeError) as exc:
                    return QMessageBox.warning(self, "복구 실패", str(exc))
        if state is None:
            return QMessageBox.information(self, "복구", "복구할 이전 변경이 없습니다.")

        current_state = self.current_persistent_state_data()
        selected_frame_id = None
        selected_row = self.frame_list_ui1.currentRow() if hasattr(self, "frame_list_ui1") else -1
        if 0 <= selected_row < len(self.frames):
            selected_frame_id = self.frames[selected_row].get("frame_id")
        change_summary = self.describe_state_restore(current_state, state)

        was_autosave_enabled = self.autosave_enabled
        self.autosave_enabled = False
        try:
            self.apply_persistent_state_data(state)
            if selected_frame_id is not None:
                restored_row = next(
                    (
                        index for index, frame in enumerate(self.frames)
                        if frame.get("frame_id") == selected_frame_id
                    ),
                    -1,
                )
                if restored_row >= 0:
                    self.frame_list_ui1.setCurrentRow(restored_row)
                    self.load_frame_to_ui(restored_row)
            with open(self.state_file_path, 'w', encoding='utf-8') as state_file:
                json.dump(state, state_file, indent=2, ensure_ascii=False)
        finally:
            self.autosave_enabled = was_autosave_enabled
        QMessageBox.information(
            self,
            "복구 완료",
            "직전 GUI 상태로 되돌렸습니다.\n\n변경 내용:\n" + change_summary,
        )

    def load_persistent_state(self):
        if not os.path.exists(self.state_file_path):
            return
        try:
            with open(self.state_file_path, 'r', encoding='utf-8') as f:
                state = json.load(f)

            self.frames = [self.normalize_frame_data(frame) for frame in state.get("frames", [])]
            for frame in self.frames:
                frame.setdefault("frame_id", uuid.uuid4().hex)
            self.saved_sequences = []
            for seq in state.get("saved_sequences", []):
                sequence = {
                    "sequence_id": str(seq.get("sequence_id") or uuid.uuid4().hex),
                    "name": str(seq.get("name", "Sequence")),
                    "max_seq_ms": int(seq.get("max_seq_ms", 5000)),
                    "repeat_count": max(1, int(seq.get("repeat_count", 1))),
                    "playback_speed": max(0.1, min(5.0, float(seq.get("playback_speed", 1.0)))),
                    "repeatable": bool(seq.get("repeatable", True)),
                    "completion": copy.deepcopy(seq.get("completion", {
                        "position_tolerance_deg": 2.0,
                        "settle_duration_ms": 80,
                        "settle_timeout_ms": 3000,
                    })),
                    "frames": [self.normalize_frame_data(frame) for frame in seq.get("frames", [])],
                }
                self.repair_legacy_sequence_timing(sequence["frames"])
                self.saved_sequences.append(sequence)
            self.motion_sequence = [self.normalize_frame_data(frame) for frame in state.get("motion_sequence", [])]
            self.repair_legacy_sequence_timing(self.motion_sequence)
            self.link_sequence_frames_to_library()
            self.max_seq_ms = int(state.get("max_seq_ms", self.max_seq_ms))
            self.loaded_sequence_id = state.get("loaded_sequence_id")
            self.default_pose_frame_id = state.get("default_pose_frame_id")
            zoom_percent = max(25, min(400, int(state.get("timeline_zoom_percent", 250))))
            self.SCALE = zoom_percent / 100.0
            if hasattr(self, "spin_timeline_zoom"):
                self.spin_timeline_zoom.blockSignals(True)
                self.spin_timeline_zoom.setValue(zoom_percent)
                self.spin_timeline_zoom.blockSignals(False)
            if hasattr(self, 'spin_max_time'):
                self.spin_max_time.setValue(self.max_seq_ms)

            self.rebuild_frame_list_ui()
            self.refresh_sequence_list()
            self.refresh_timeline_ui()
            self.refresh_loaded_sequence_indicator()
            print(f"[✅ 자동 로드] 프레임 {len(self.frames)}개, 시퀀스 {len(self.saved_sequences)}개를 복원했습니다.")
        except Exception as e:
            print(f"[⚠️ 자동 로드 실패] {type(e).__name__}: {e}")

    def link_sequence_frames_to_library(self):
        """이전 저장 데이터의 시퀀스 프레임을 원본 라이브러리와 연결합니다."""
        by_id = {frame.get("frame_id"): frame for frame in self.frames if frame.get("frame_id")}
        by_name = {}
        for frame in self.frames:
            by_name.setdefault(frame.get("name"), []).append(frame)

        collections = [self.motion_sequence]
        collections.extend(seq.get("frames", []) for seq in self.saved_sequences)
        for frames in collections:
            for sequence_frame in frames:
                frame_id = sequence_frame.get("frame_id")
                if frame_id in by_id:
                    sequence_frame["source_frame_id"] = frame_id
                    continue
                matches = by_name.get(sequence_frame.get("name"), [])
                if len(matches) == 1:
                    sequence_frame["frame_id"] = matches[0]["frame_id"]
                    sequence_frame["source_frame_id"] = matches[0]["frame_id"]

    def load_3d_robot_urdf(self):
        self.urdf_loaded = False
        self.urdf_error = ""
        self.urdf_viewers = []
        self.robot_model = None
        urdf_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "step.urdf")
        try:
            # FK에는 메시가 필요 없으므로 용량이 큰 STL은 로드하지 않고
            # 링크/조인트 트리와 origin/axis만 로드합니다.
            self.robot_model = SimpleURDFModel.load(urdf_path, load_visuals=False)
            self.urdf_loaded = True
            print(
                "[FK 로딩 완료] "
                f"root={self.robot_model.root_link}, "
                f"end-effectors={self.robot_model.end_effector_links}"
            )
        except Exception as exc:
            self.urdf_error = str(exc)
            print(f"[❌ FK 로딩 실패] {urdf_path}: {exc}")

    def init_3d_viewer(self):
        blank = QWidget()
        blank.setMinimumWidth(200)
        blank.setMinimumHeight(300)
        blank.setStyleSheet("background: transparent; border: none;")
        return blank

    def update_3d_robot(self, temp_angles=None):
        return

    def initUI(self):
        self.setWindowTitle('IRC STEP SDK MOTION made by geonwoo')
        screen = QApplication.primaryScreen().availableGeometry()
        window_width = int(screen.width() * 0.94)
        window_height = int(screen.height() * 0.90)
        self.setGeometry(
            screen.x() + (screen.width() - window_width) // 2,
            screen.y() + (screen.height() - window_height) // 2,
            window_width,
            window_height,
        )
        # Ubuntu 기본 테마에서는 얇은 스크롤바의 손잡이와 배경 구분이
        # 약하므로 모든 탭에서 같은 저채도 청록색 손잡이를 사용합니다.
        self.setStyleSheet(
            "QScrollBar:vertical { background: #e3e8eb; width: 17px; margin: 1px; }"
            "QScrollBar::handle:vertical { background: #607d8b; min-height: 34px; border-radius: 7px; margin: 2px; }"
            "QScrollBar::handle:vertical:hover { background: #376b7a; }"
            "QScrollBar:horizontal { background: #e3e8eb; height: 17px; margin: 1px; }"
            "QScrollBar::handle:horizontal { background: #607d8b; min-width: 34px; border-radius: 7px; margin: 2px; }"
            "QScrollBar::handle:horizontal:hover { background: #376b7a; }"
            "QScrollBar::add-line, QScrollBar::sub-line { width: 0px; height: 0px; }"
            "QScrollBar::add-page, QScrollBar::sub-page { background: transparent; }"
        )
        main_layout = QVBoxLayout()
        self.lbl_conn_status = QLabel("⚠️ 다이나믹셀 하드웨어 포트 연결 대기 중...")
        self.lbl_conn_status.setAlignment(Qt.AlignCenter)
        self.lbl_conn_status.setStyleSheet("font-weight: bold; font-size: 14pt; background-color: #333; color: #FFC107; padding: 8px; border-radius: 5px; margin-bottom: 5px;")
        main_layout.addWidget(self.lbl_conn_status)
        recovery_layout = QHBoxLayout()
        recovery_layout.addStretch(1)
        self.btn_undo_state = QPushButton("↩ 직전 변경 복구 (Ctrl+Z)")
        self.btn_undo_state.setStyleSheet(
            "background-color: #455a64; color: white; font-weight: bold; min-height: 34px;"
        )
        self.btn_undo_state.clicked.connect(self.undo_last_gui_change)
        recovery_layout.addWidget(self.btn_undo_state)
        main_layout.addLayout(recovery_layout)
        self.undo_shortcut = QShortcut(QKeySequence.Undo, self)
        self.undo_shortcut.activated.connect(self.undo_last_gui_change)
        self.tabs = QTabWidget()
        self.tabs.tabBar().setExpanding(True)
        self.tabs.tabBar().setUsesScrollButtons(False)
        self.tabs.setStyleSheet(
            "QTabBar::tab { min-height: 50px; min-width: 360px; "
            "font-size: 14pt; font-weight: bold; padding: 10px 18px; }"
        )
        self.tab_frame = QWidget()
        self.init_frame_tab(self.tab_frame)
        self.tab_motion = QWidget()
        self.init_motion_tab(self.tab_motion)
        self.tab_sequence_composer = QWidget()
        self.init_sequence_composer_tab(self.tab_sequence_composer)
        self.tabs.addTab(self.tab_frame, "🎬 1. 단일 프레임 제작 (관절 제어)")
        self.tabs.addTab(self.tab_motion, "🎞️ 2. 모션 제작 (프레임 이어붙이기)")
        self.tabs.addTab(self.tab_sequence_composer, "🧩 3. 시퀀스 조합 (시퀀스 이어붙이기)")
        main_layout.addWidget(self.tabs)
        self.setLayout(main_layout)

    def init_frame_tab(self, tab):
        layout = QHBoxLayout()
        left_panel = QVBoxLayout()
        
        torque_master_layout = QHBoxLayout()
        self.btn_all_torque_on = QPushButton("✅ 전체 토크 ON (잠금)")
        self.btn_all_torque_on.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; font-size: 13pt; min-height: 40px;")
        self.btn_all_torque_on.clicked.connect(self.set_all_torque_on)
        
        self.btn_all_torque_off = QPushButton("❌ 전체 토크 OFF (티칭)")
        self.btn_all_torque_off.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; font-size: 13pt; min-height: 40px;")
        self.btn_all_torque_off.clicked.connect(self.set_all_torque_off)

        self.btn_rescan_motors = QPushButton("🔄 모터 재탐색")
        self.btn_rescan_motors.setStyleSheet("background-color: #17a2b8; color: white; font-weight: bold; font-size: 13pt; min-height: 40px;")
        self.btn_rescan_motors.clicked.connect(self.rescan_online_motors)
        
        torque_master_layout.addWidget(self.btn_all_torque_on)
        torque_master_layout.addWidget(self.btn_all_torque_off)
        torque_master_layout.addWidget(self.btn_rescan_motors)
        left_panel.addLayout(torque_master_layout)

        torque_group_layout = QHBoxLayout()
        for group_name, joint_ids in self.torque_groups.items():
            btn_group = QPushButton(f"{group_name} 전체 OFF")
            btn_group.setCheckable(True)
            btn_group.setMinimumHeight(36)
            btn_group.setStyleSheet(
                "background-color: #dc3545; color: white; font-weight: bold; font-size: 11pt;"
            )
            btn_group.toggled.connect(
                lambda checked, name=group_name, ids=joint_ids: self.sync_torque_group(name, ids, checked)
            )
            self.torque_group_btns[group_name] = btn_group
            torque_group_layout.addWidget(btn_group)
        left_panel.addLayout(torque_group_layout)

        scroll_area = QScrollArea()
        scroll_area.setWidgetResizable(True)
        scroll_content = QWidget()
        joint_layout = QVBoxLayout()

        for joint in self.joint_data:
            row_frame = QFrame()
            row_frame.setFrameShape(QFrame.StyledPanel)
            row_layout = QHBoxLayout(row_frame)
            lbl_id = QLabel(f"[{joint['id']}]")
            lbl_id.setMinimumWidth(45)
            lbl_id.setStyleSheet("font-weight: bold; font-size: 13pt; color: #0055A4;")
            color = "#FF8C00" if joint["type"] == "28" else "#228B22" if joint["type"] == "64" else "#333333"
            lbl_name = QLabel(f"{joint['name']}\n(MX-{joint['type']})")
            lbl_name.setMinimumWidth(140)
            lbl_name.setStyleSheet(f"color: {color}; font-weight: bold; font-size: 12pt;")
            
            slider = NoWheelSlider(Qt.Horizontal)
            slider.setRange(-180, 180)
            slider.setMinimumWidth(160) 
            slider.setEnabled(False) 
            
            spinbox = NoWheelSpinBox()
            spinbox.setRange(-180, 180)
            spinbox.setMinimumWidth(70)
            spinbox.setMinimumHeight(40)
            spinbox.setStyleSheet("font-size: 13pt; font-weight: bold;")
            spinbox.setEnabled(False) 
            
            btn_torque = QPushButton("OFF")
            btn_torque.setCheckable(True)
            btn_torque.setChecked(False) # 🚨 디폴트를 OFF(False)로 설정하여 안전하게 기동!
            btn_torque.setMinimumSize(50, 40)
            btn_torque.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold;")
            
            # 🔥 GUI 작동 증명: 람다(lambda)를 통해 완벽하게 연결되어 있습니다.
            btn_torque.toggled.connect(lambda checked, j_id=joint['id']: self.sync_torque(j_id, checked))
            slider.valueChanged.connect(lambda val, j_id=joint['id']: self.sync_values(j_id, val, 'slider'))
            # 숫자를 입력하는 동안에는 GUI만 갱신하고, Enter를 눌렀을 때만
            # 최종 값을 모터로 전송합니다.
            spinbox.valueChanged.connect(
                lambda val, j_id=joint['id']: self.sync_values(j_id, val, 'spinbox', transmit_motor=False)
            )
            spinbox.lineEdit().returnPressed.connect(
                lambda j_id=joint['id'], widget=spinbox: self.sync_values(
                    j_id, widget.value(), 'spinbox', transmit_motor=True
                )
            )
            
            self.sliders[joint['id']] = slider
            self.spinboxes[joint['id']] = spinbox
            self.torque_btns[joint['id']] = btn_torque
            
            row_layout.addWidget(lbl_id)
            row_layout.addWidget(lbl_name)
            row_layout.addWidget(slider, stretch=1)
            row_layout.addWidget(spinbox)
            row_layout.addWidget(btn_torque)
            joint_layout.addWidget(row_frame)
        
        scroll_content.setLayout(joint_layout)
        scroll_area.setWidget(scroll_content)
        left_panel.addWidget(scroll_area)
        layout.addLayout(left_panel, 2) 

        # 기존 URDF/3D 뷰어를 제거한 빈 영역은 프레임 목록에
        # 전부 할당합니다. 남은 코드의 호환성을 위해 속성만 유지합니다.
        self.canvas_3d = None

        frame_group = QGroupBox("저장된 프레임 목록")
        frame_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        frame_outer_layout = QHBoxLayout()
        frame_controls_widget = QWidget()
        frame_controls_widget.setMinimumWidth(330)
        frame_controls_widget.setMaximumWidth(430)
        frame_layout = QVBoxLayout(frame_controls_widget)

        self.btn_read_robot = QPushButton('🤖 물리 로봇 실제 관절값 불러오기\n(토크 OFF 후 손으로 꺾고 누르세요)')
        self.btn_read_robot.setStyleSheet("background-color: #FFC107; color: #333; font-weight: bold; font-size: 13pt; min-height: 50px;")
        self.btn_read_robot.clicked.connect(self.read_angles_from_robot)
        frame_layout.addWidget(self.btn_read_robot)

        time_layout = QHBoxLayout()
        time_label = QLabel("이동 시간(ms):")
        self.time_spinbox = QSpinBox()
        self.time_spinbox.setRange(10, 5000)
        self.time_spinbox.setValue(500)
        self.time_spinbox.setSingleStep(50)
        self.time_spinbox.setMinimumHeight(45)
        time_layout.addWidget(time_label)
        time_layout.addWidget(self.time_spinbox)
        frame_layout.addLayout(time_layout)

        self.btn_add = QPushButton('+ 프레임 추가')
        self.btn_add.setStyleSheet("background-color: #E6F0FA; font-weight: bold; font-size: 14pt; min-height: 50px;")
        self.btn_add.clicked.connect(self.add_frame)
        frame_layout.addWidget(self.btn_add)

        self.btn_load_frame = QPushButton('📂 프레임 불러오기 (각도 바로 적용)')
        self.btn_load_frame.setStyleSheet(
            "background-color: #007bff; color: white; font-weight: bold; font-size: 14pt; min-height: 50px;"
        )
        self.btn_load_frame.clicked.connect(self.apply_selected_frame)
        frame_layout.addWidget(self.btn_load_frame)

        self.btn_execute = QPushButton('▶️ 단일 실행')
        self.btn_execute.setStyleSheet("background-color: #FF5722; color: white; font-weight: bold; font-size: 14pt; min-height: 50px;")
        self.btn_execute.clicked.connect(self.execute_frame)
        frame_layout.addWidget(self.btn_execute)
        
        grid_btn_layout = QGridLayout()
        btn_style = "font-size: 12pt; min-height: 40px; font-weight: bold;"
        
        self.btn_update = QPushButton('💾 재저장')
        self.btn_update.setStyleSheet(btn_style)
        self.btn_update.clicked.connect(self.update_frame)
        self.btn_rename = QPushButton('✏️ 이름 변경')
        self.btn_rename.setStyleSheet(btn_style)
        self.btn_rename.clicked.connect(self.rename_frame)
        self.btn_mirror = QPushButton('🔄 좌우반전')
        self.btn_mirror.setStyleSheet(btn_style)
        self.btn_mirror.clicked.connect(self.mirror_frame)
        
        delete_layout = QHBoxLayout()
        self.btn_toggle_select = QPushButton('☑️ 선택')
        self.btn_toggle_select.setCheckable(True)
        self.btn_toggle_select.setStyleSheet(btn_style)
        self.btn_toggle_select.toggled.connect(self.toggle_select_mode)
        self.btn_delete = QPushButton('🗑️ 삭제')
        self.btn_delete.setStyleSheet(btn_style)
        self.btn_delete.clicked.connect(self.delete_frame)
        delete_layout.addWidget(self.btn_toggle_select)
        delete_layout.addWidget(self.btn_delete)
        
        grid_btn_layout.addWidget(self.btn_update, 0, 0)
        grid_btn_layout.addWidget(self.btn_rename, 0, 1)
        grid_btn_layout.addWidget(self.btn_mirror, 1, 0)
        grid_btn_layout.addLayout(delete_layout, 1, 1)
        frame_layout.addLayout(grid_btn_layout)
        frame_layout.addStretch(1)

        self.frame_list_ui1 = QListWidget()
        self.frame_list_ui1.setStyleSheet(
            "QListWidget { background-color: #ffffff; color: #212121; }"
            "QListWidget::item:selected { background-color: #90caf9; color: #111111; }"
        )
        self.frame_list_ui1.itemSelectionChanged.connect(self.sync_drag_selection)
        self.frame_list_ui1.currentRowChanged.connect(
            self.select_full_frame_for_editing
        )
        self.frame_list_ui1.currentItemChanged.connect(
            self.update_frame_library_selection_style
        )

        self.frame_favorites_ui = QListWidget()
        self.frame_favorites_ui.setStyleSheet(
            "QListWidget { background-color: #ffffff; color: #212121; }"
            "QListWidget::item:selected { background-color: #ffd54f; color: #111111; }"
        )
        self.frame_favorites_ui.setSelectionMode(QAbstractItemView.SingleSelection)
        self.frame_favorites_ui.itemClicked.connect(
            self.select_favorite_frame_for_editing
        )
        self.frame_favorites_ui.itemDoubleClicked.connect(
            self.select_favorite_frame_for_editing
        )
        self.btn_remove_frame_favorite = QPushButton("☆ 선택 프레임 즐겨찾기 해제")
        self.btn_remove_frame_favorite.clicked.connect(
            self.remove_selected_frame_favorite
        )
        favorites_page = QWidget()
        favorites_layout = QVBoxLayout(favorites_page)
        favorites_layout.setContentsMargins(0, 4, 0, 0)
        favorites_layout.addWidget(self.frame_favorites_ui)
        favorites_layout.addWidget(self.btn_remove_frame_favorite)

        self.frame_library_tabs = QTabWidget()
        self.frame_library_tabs.setMinimumWidth(620)
        self.frame_library_tabs.tabBar().setExpanding(True)
        self.frame_library_tabs.tabBar().setUsesScrollButtons(False)
        self.frame_library_tabs.setStyleSheet(
            "QTabBar::tab { min-width: 250px; min-height: 44px; "
            "font-size: 13pt; font-weight: bold; padding: 8px 18px; }"
            "QTabBar::tab:selected { background-color: #1976d2; color: white; }"
            "QTabBar::tab:!selected { background-color: #e3f2fd; color: #263238; }"
        )
        self.frame_library_tabs.addTab(self.frame_list_ui1, "전체 프레임")
        self.frame_library_tabs.addTab(favorites_page, "★ 즐겨찾기 (0)")
        self.frame_library_tabs.currentChanged.connect(
            self.on_frame_library_tab_changed
        )

        # 넓고 높은 목록을 왼쪽, 실행/저장 컨트롤을 오른쪽에 배치합니다.
        frame_outer_layout.addWidget(self.frame_library_tabs, 3)
        frame_outer_layout.addWidget(frame_controls_widget, 1)
        frame_group.setLayout(frame_outer_layout)
        layout.addWidget(frame_group, 3)
        tab.setLayout(layout)

    def init_motion_tab(self, tab):
        # 타임라인과 관절 궤적을 추가해도 다른 탭의 최소 높이를 화면 아래로
        # 밀어내지 않도록 모션 제작 페이지 전체를 세로 스크롤 영역에 넣습니다.
        outer_layout = QVBoxLayout(tab)
        outer_layout.setContentsMargins(0, 0, 0, 0)
        motion_scroll = QScrollArea()
        motion_scroll.setWidgetResizable(True)
        motion_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        motion_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        motion_content = QWidget()
        main_layout = QVBoxLayout(motion_content)
        motion_scroll.setWidget(motion_content)
        outer_layout.addWidget(motion_scroll)
        self.motion_page_scroll = motion_scroll

        top_layout = QHBoxLayout()
        
        lib_group = QGroupBox("📚 라이브러리 (드래그 앤 드롭 가능!)")
        lib_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        lib_layout = QHBoxLayout() 
        
        sequence_layout = QVBoxLayout()
        lbl_sequence = QLabel("🎞️ 저장된 시퀀스")
        lbl_sequence.setStyleSheet("color: #6f42c1;")
        
        self.sequence_list_ui = QListWidget()
        self.sequence_list_ui.setMinimumHeight(300)
        self.sequence_list_ui.setSelectionMode(QAbstractItemView.SingleSelection)
        self.sequence_list_ui.itemDoubleClicked.connect(self.load_sequence_from_list_item)
        
        sequence_layout.addWidget(lbl_sequence)
        self.lbl_loaded_sequence = QLabel("현재 타임라인: 새 시퀀스")
        self.lbl_loaded_sequence.setStyleSheet("color: #00897b; font-weight: bold;")
        sequence_layout.addWidget(self.lbl_loaded_sequence)
        sequence_layout.addWidget(self.sequence_list_ui)
        self.btn_load_saved_sequence = QPushButton("📥 선택 시퀀스를 타임라인에 불러오기")
        self.btn_load_saved_sequence.setStyleSheet(
            "background-color: #6f42c1; color: white; font-weight: bold;"
        )
        self.btn_load_saved_sequence.clicked.connect(self.load_selected_sequence_to_timeline)
        sequence_layout.addWidget(self.btn_load_saved_sequence)
        self.btn_rename_saved_sequence = QPushButton("✏️ 선택 시퀀스 이름 변경")
        self.btn_rename_saved_sequence.setStyleSheet(
            "background-color: #00897b; color: white; font-weight: bold;"
        )
        self.btn_rename_saved_sequence.clicked.connect(
            self.rename_selected_saved_sequence
        )
        sequence_layout.addWidget(self.btn_rename_saved_sequence)
        self.btn_delete_saved_sequence = QPushButton("🗑️ 선택 시퀀스 삭제")
        self.btn_delete_saved_sequence.setStyleSheet(
            "background-color: #dc3545; color: white; font-weight: bold;"
        )
        self.btn_delete_saved_sequence.clicked.connect(self.delete_selected_saved_sequence)
        sequence_layout.addWidget(self.btn_delete_saved_sequence)
        
        all_layout = QVBoxLayout()
        self.lbl_all_frames = QLabel("📁 전체 프레임")
        self.btn_favorite_frames_only = QPushButton("☆ 즐겨찾기만 보기")
        self.btn_favorite_frames_only.setCheckable(True)
        self.btn_favorite_frames_only.setStyleSheet(
            "QPushButton { font-weight: bold; padding: 6px; }"
            "QPushButton:checked { background-color: #f9a825; color: white; }"
        )
        self.btn_favorite_frames_only.toggled.connect(
            self.toggle_favorite_frames_filter
        )
        self.frame_list_all = QListWidget()
        self.frame_list_all.setMinimumHeight(300)
        self.frame_list_all.setSelectionMode(QAbstractItemView.ExtendedSelection)
        self.frame_list_all.setDragEnabled(True)
        self.frame_list_all.itemSelectionChanged.connect(self.on_all_selected)
        self.frame_list_all.itemDoubleClicked.connect(
            self.open_frame_editor_from_library_item
        )
        
        all_layout.addWidget(self.lbl_all_frames)
        all_layout.addWidget(self.btn_favorite_frames_only)
        all_layout.addWidget(self.frame_list_all)
        self.btn_load_motion_library_frame = QPushButton(
            "📂 선택 프레임 불러오기 (각도 바로 적용)"
        )
        self.btn_load_motion_library_frame.setStyleSheet(
            "background-color: #1976d2; color: white; font-weight: bold; padding: 8px;"
        )
        self.btn_load_motion_library_frame.clicked.connect(
            self.load_selected_motion_library_frame
        )
        all_layout.addWidget(self.btn_load_motion_library_frame)
        self.btn_set_default_pose = QPushButton(
            "🏠 선택 프레임을 기본자세로 설정"
        )
        self.btn_set_default_pose.setStyleSheet(
            "background-color: #546e7a; color: white; font-weight: bold; padding: 8px;"
        )
        self.btn_set_default_pose.clicked.connect(
            self.set_selected_frame_as_default_pose
        )
        all_layout.addWidget(self.btn_set_default_pose)

        lib_layout.addLayout(sequence_layout)
        lib_layout.addLayout(all_layout)
        lib_group.setLayout(lib_layout)
        
        mid_layout = QVBoxLayout()
        mid_layout.setAlignment(Qt.AlignCenter)
        self.btn_add_to_motion = QPushButton('타임라인에\n추가 ⬇️')
        self.btn_add_to_motion.setMinimumSize(100, 70)
        self.btn_add_to_motion.setStyleSheet("font-weight: bold; font-size: 13pt; background-color: #007bff; color: white;")
        self.btn_add_to_motion.clicked.connect(self.add_to_motion)
        mid_layout.addWidget(self.btn_add_to_motion)
        
        # 제거한 URDF 뷰어가 차지하던 빈 열을 없애고 라이브러리가 가로폭을
        # 모두 사용하게 합니다. 저장 시퀀스와 프레임 목록도 동시에 커집니다.
        top_layout.addWidget(lib_group, 9)
        top_layout.addLayout(mid_layout, 1)
        
        ctrl_group = QGroupBox("⚙️ 시퀀스 컨트롤 패널")
        ctrl_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        ctrl_layout = QHBoxLayout()
        
        self.btn_keyframe_play = QPushButton('▶️ 재생')
        self.btn_keyframe_play.setStyleSheet("background-color: #ff9800; color: white; min-height: 40px; font-weight: bold;")
        self.btn_keyframe_play.clicked.connect(self.play_motion_page_sequence)

        self.btn_keyframe_pause = QPushButton('⏸️ 일시정지')
        self.btn_keyframe_pause.setStyleSheet("background-color: #607d8b; color: white; min-height: 40px; font-weight: bold;")
        self.btn_keyframe_pause.clicked.connect(self.pause_motion_sequence)
        self.btn_keyframe_pause.setEnabled(False)

        self.btn_stop_motion = QPushButton('⏹️ 정지')
        self.btn_stop_motion.setStyleSheet("background-color: #dc3545; color: white; min-height: 40px;")
        self.btn_stop_motion.clicked.connect(self.stop_motion_sequence)
        self.btn_stop_motion.setEnabled(False)

        self.btn_robot_sync = QPushButton('🤖 로봇 동기화: OFF')
        self.btn_robot_sync.setCheckable(True)
        self.btn_robot_sync.setStyleSheet(
            "QPushButton { background-color: #555; color: white; min-height: 40px; font-weight: bold; }"
            "QPushButton:checked { background-color: #e91e63; }"
        )
        self.btn_robot_sync.toggled.connect(self.toggle_robot_sync)

        repeat_label = QLabel("반복:")
        self.spin_motion_repeat = QSpinBox()
        self.spin_motion_repeat.setRange(1, 999)
        self.spin_motion_repeat.setValue(1)
        self.spin_motion_repeat.setSuffix(" 회")
        self.spin_motion_repeat.setMinimumWidth(85)
        speed_label = QLabel("배속:")
        self.spin_motion_speed = QDoubleSpinBox()
        self.spin_motion_speed.setRange(0.1, 5.0)
        self.spin_motion_speed.setDecimals(2)
        self.spin_motion_speed.setSingleStep(0.05)
        self.spin_motion_speed.setValue(1.0)
        self.spin_motion_speed.setSuffix("x")
        self.spin_motion_speed.setMinimumWidth(90)

        self.btn_default_pose = QPushButton('🏠 기본자세 복귀 (천천히)')
        self.btn_default_pose.setStyleSheet(
            "background-color: #6f42c1; color: white; min-height: 40px; font-weight: bold;"
        )
        self.btn_default_pose.clicked.connect(self.return_to_default_pose)

        self.btn_clear_motion = QPushButton('🧹 초기화')
        self.btn_clear_motion.clicked.connect(self.clear_motion)
        self.btn_save_sequence = QPushButton('💾 저장')
        self.btn_save_sequence.setStyleSheet("background-color: #17a2b8; color: white;")
        self.btn_save_sequence.clicked.connect(self.save_sequence)
        self.btn_update_sequence = QPushButton('♻️ 현재 시퀀스 재저장')
        self.btn_update_sequence.setStyleSheet(
            "background-color: #6f42c1; color: white; font-weight: bold;"
        )
        self.btn_update_sequence.clicked.connect(self.update_loaded_sequence)
        self.btn_update_sequence.setEnabled(False)
        self.btn_manage_seq = QPushButton('📁 관리/불러오기')
        self.btn_manage_seq.clicked.connect(self.open_sequence_manager)
        self.btn_export = QPushButton('🚀 Jetson 내보내기')
        self.btn_export.setStyleSheet("background-color: #28a745; color: white;")
        self.btn_export.clicked.connect(self.export_motion_json)
        self.btn_export_all_motions = QPushButton('💾 전체 모션 JSON 저장')
        self.btn_export_all_motions.clicked.connect(self.export_all_motions_json)
        
        ctrl_layout.addWidget(self.btn_keyframe_play)
        ctrl_layout.addWidget(self.btn_keyframe_pause)
        ctrl_layout.addWidget(self.btn_stop_motion)
        ctrl_layout.addWidget(self.btn_robot_sync)
        ctrl_layout.addWidget(repeat_label)
        ctrl_layout.addWidget(self.spin_motion_repeat)
        ctrl_layout.addWidget(speed_label)
        ctrl_layout.addWidget(self.spin_motion_speed)
        ctrl_layout.addWidget(self.btn_default_pose)
        ctrl_layout.addSpacing(15)
        ctrl_layout.addWidget(self.btn_clear_motion)
        ctrl_layout.addWidget(self.btn_save_sequence)
        ctrl_layout.addWidget(self.btn_update_sequence)
        ctrl_layout.addWidget(self.btn_manage_seq)
        ctrl_layout.addSpacing(15)
        ctrl_layout.addWidget(self.btn_export)
        ctrl_layout.addWidget(self.btn_export_all_motions)
        ctrl_group.setLayout(ctrl_layout)

        timeline_group = QGroupBox("🎬 프레임 타임라인 (배경 드래그: 시간탐색 / 라이브러리에서 끌어놓기 가능!)")
        timeline_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        tl_base_layout = QVBoxLayout()
        
        tl_tools_layout = QHBoxLayout()
        self.lbl_total_time = QLabel("현재 모션 종료 지점: 0ms / 타임라인 길이: 5000ms")
        self.lbl_total_time.setStyleSheet("color: #69f0ae;")
        
        bright_spinbox_style = "background-color: #ffffff; color: #000000; font-weight: bold; border: 1px solid #aaa; font-size: 12pt;"
        
        lbl_max_time = QLabel("⏳ 타임라인 총 길이 설정(ms):")
        self.spin_max_time = QSpinBox()
        self.spin_max_time.setRange(1000, 60000)
        self.spin_max_time.setValue(self.max_seq_ms)
        self.spin_max_time.setSingleStep(500)
        self.spin_max_time.setStyleSheet(bright_spinbox_style)

        lbl_zoom = QLabel("   |   타임라인 확대(%):")
        self.spin_timeline_zoom = QSpinBox()
        self.spin_timeline_zoom.setRange(25, 400)
        self.spin_timeline_zoom.setSingleStep(25)
        self.spin_timeline_zoom.setValue(int(self.SCALE * 100))
        self.spin_timeline_zoom.setSuffix(" %")
        self.spin_timeline_zoom.setStyleSheet(bright_spinbox_style)
        self.btn_apply_timeline_zoom = QPushButton("확대 적용")
        self.btn_apply_timeline_zoom.setStyleSheet(
            "background-color: #6f42c1; color: white; font-weight: bold;"
        )
        self.btn_apply_timeline_zoom.clicked.connect(self.apply_timeline_zoom)
        self.btn_auto_timeline_zoom = QPushButton("프레임 시간에 자동 맞춤")
        self.btn_auto_timeline_zoom.setStyleSheet(
            "background-color: #00897b; color: white; font-weight: bold;"
        )
        self.btn_auto_timeline_zoom.setToolTip(
            "프레임 시간의 중앙값을 기준으로 화면 확대율과 "
            "시간 눈금 간격을 자동으로 맞춥니다. 실제 프레임 시간은 변경하지 않습니다."
        )
        self.btn_auto_timeline_zoom.clicked.connect(
            self.auto_fit_timeline_to_frame_times
        )
        
        self.btn_apply_max_time = QPushButton("적용")
        self.btn_apply_max_time.setStyleSheet("background-color: #6f42c1; color: white;")
        self.btn_apply_max_time.clicked.connect(self.apply_max_sequence_time)
        
        lbl_target = QLabel("   |   시간 일괄 스케일링(ms):")
        self.spin_target_time = QSpinBox()
        self.spin_target_time.setRange(10, 60000)
        self.spin_target_time.setValue(1000)
        self.spin_target_time.setStyleSheet(bright_spinbox_style)
        
        self.btn_apply_time = QPushButton("비율 적용")
        self.btn_apply_time.setStyleSheet("background-color: #6f42c1; color: white;")
        self.btn_apply_time.clicked.connect(self.apply_target_time)
        
        tl_tools_layout.addWidget(self.lbl_total_time)
        tl_tools_layout.addStretch()
        tl_tools_layout.addWidget(lbl_max_time)
        tl_tools_layout.addWidget(self.spin_max_time)
        tl_tools_layout.addWidget(self.btn_apply_max_time)
        tl_tools_layout.addWidget(lbl_zoom)
        tl_tools_layout.addWidget(self.spin_timeline_zoom)
        tl_tools_layout.addWidget(self.btn_apply_timeline_zoom)
        tl_tools_layout.addWidget(self.btn_auto_timeline_zoom)
        tl_tools_layout.addWidget(lbl_target)
        tl_tools_layout.addWidget(self.spin_target_time)
        tl_tools_layout.addWidget(self.btn_apply_time)
        tl_base_layout.addLayout(tl_tools_layout)

        gap_tools_layout = QHBoxLayout()
        self.btn_capture_timeline_pose = QPushButton("📌 현재 시점 자세를 프레임 제작으로 가져오기")
        self.btn_capture_timeline_pose.setStyleSheet(
            "background-color: #1565c0; color: white; font-weight: bold;"
        )
        self.btn_capture_timeline_pose.clicked.connect(
            self.capture_timeline_pose_to_frame_editor
        )
        gap_tools_layout.addWidget(self.btn_capture_timeline_pose)
        gap_tools_layout.addStretch()
        gap_label = QLabel("↔️ 모든 프레임 사이 동일 간격(ms):")
        self.spin_uniform_frame_gap = QSpinBox()
        self.spin_uniform_frame_gap.setRange(0, 10000)
        self.spin_uniform_frame_gap.setSingleStep(10)
        self.spin_uniform_frame_gap.setValue(0)
        self.spin_uniform_frame_gap.setSuffix(" ms")
        self.spin_uniform_frame_gap.setStyleSheet(bright_spinbox_style)
        self.spin_uniform_frame_gap.lineEdit().returnPressed.connect(self.apply_uniform_frame_gap)
        self.btn_apply_uniform_gap = QPushButton("간격 일괄 적용")
        self.btn_apply_uniform_gap.setStyleSheet("background-color: #00897b; color: white;")
        self.btn_apply_uniform_gap.clicked.connect(self.apply_uniform_frame_gap)
        gap_tools_layout.addWidget(gap_label)
        gap_tools_layout.addWidget(self.spin_uniform_frame_gap)
        gap_tools_layout.addWidget(self.btn_apply_uniform_gap)
        tl_base_layout.addLayout(gap_tools_layout)
        
        self.timeline_scroll = QScrollArea()
        self.timeline_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.timeline_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.timeline_scroll.setMinimumHeight(275)
        
        self.timeline_container = TimelineContainer(self)
        self.timeline_scroll.setWidget(self.timeline_container)
        
        tl_base_layout.addWidget(self.timeline_scroll)

        trajectory_header = QHBoxLayout()
        trajectory_label = QLabel(
            "📈 단일 관절 연속 궤적 — 모든 프레임을 하나의 선으로 연결"
        )
        trajectory_label.setStyleSheet("color: #90caf9; font-weight: bold;")
        self.combo_trajectory_group = QComboBox()
        default_joint_index = 0
        for index, joint in enumerate(self.joint_data):
            joint_id = int(joint["id"])
            self.combo_trajectory_group.addItem(
                f"[{joint_id}] {joint['name']}",
                joint_id,
            )
            if joint_id == 22:
                default_joint_index = index
        self.combo_trajectory_group.setCurrentIndex(default_joint_index)
        self.combo_trajectory_group.setMinimumWidth(230)
        self.combo_trajectory_group.setStyleSheet(
            "background: white; color: black; font-weight: bold;"
        )
        trajectory_header.addWidget(trajectory_label)
        trajectory_header.addStretch()
        trajectory_header.addWidget(QLabel("한 선으로 볼 관절:"))
        trajectory_header.addWidget(self.combo_trajectory_group)
        tl_base_layout.addLayout(trajectory_header)

        self.trajectory_graph = JointTrajectoryWidget(self)
        self.combo_trajectory_group.currentTextChanged.connect(
            lambda _group_name: self.trajectory_graph.update()
        )
        tl_base_layout.addWidget(self.trajectory_graph)
        timeline_group.setLayout(tl_base_layout)
        
        main_layout.addLayout(top_layout, 5)
        main_layout.addWidget(ctrl_group, 1)
        main_layout.addWidget(timeline_group, 3)
        
        # tab에는 outer_layout이 이미 생성자 인자로 설정되어 있습니다.

    def init_sequence_composer_tab(self, tab):
        # 내부 프레임 이름과 시간이 보이도록 기본을 150%로 확대합니다.
        self.COMPOSER_SCALE = 1.5
        self.sequence_composer_entries = []
        main_layout = QVBoxLayout(tab)

        title = QLabel("🧩 저장된 시퀀스를 이어붙여 새로운 시퀀스 만들기")
        title.setStyleSheet("font-size: 16pt; font-weight: bold; padding: 8px;")
        main_layout.addWidget(title)

        source_group = QGroupBox("저장된 시퀀스")
        source_layout = QVBoxLayout(source_group)
        self.composer_source_list = QListWidget()
        self.composer_source_list.setSelectionMode(QAbstractItemView.SingleSelection)
        self.composer_source_list.setDragEnabled(True)
        self.composer_source_list.itemDoubleClicked.connect(lambda _: self.add_sequence_to_composer())
        source_layout.addWidget(self.composer_source_list)

        add_button = QPushButton("선택 시퀀스 추가 ➡️")
        add_button.setMinimumHeight(45)
        add_button.setStyleSheet("background-color: #007bff; color: white; font-weight: bold;")
        # QPushButton.clicked가 전달하는 checked(bool)를 시퀀스 인덱스로
        # 오인하면 False == 0이 되어 항상 첫 번째 '기본'만 추가됩니다.
        add_button.clicked.connect(
            lambda _checked=False: self.add_sequence_to_composer()
        )
        source_layout.addWidget(add_button)
        delete_source_button = QPushButton("🗑️ 선택 시퀀스 삭제")
        delete_source_button.setStyleSheet(
            "background-color: #dc3545; color: white; font-weight: bold;"
        )
        delete_source_button.clicked.connect(self.delete_composer_selected_sequence)
        source_layout.addWidget(delete_source_button)

        chain_group = QGroupBox("시퀀스 타임라인 (블록 드래그로 순서 변경 / 저장 시퀀스 끌어놓기)")
        chain_layout = QVBoxLayout(chain_group)
        composer_tools = QHBoxLayout()
        self.lbl_composer_meta = QLabel("구성: 0개 시퀀스 / 0개 프레임 / 0ms")
        self.lbl_composer_meta.setStyleSheet(
            "font-size: 13pt; font-weight: bold; color: #69f0ae; padding: 8px;"
        )
        composer_zoom_label = QLabel("타임라인 확대:")
        self.spin_composer_zoom = QSpinBox()
        self.spin_composer_zoom.setRange(25, 400)
        self.spin_composer_zoom.setSingleStep(25)
        self.spin_composer_zoom.setValue(150)
        self.spin_composer_zoom.setSuffix(" %")
        self.spin_composer_zoom.valueChanged.connect(self.apply_composer_timeline_zoom)
        composer_tools.addWidget(self.lbl_composer_meta)
        composer_tools.addStretch()
        composer_tools.addWidget(composer_zoom_label)
        composer_tools.addWidget(self.spin_composer_zoom)
        chain_layout.addLayout(composer_tools)
        self.composer_timeline_scroll = QScrollArea()
        self.composer_timeline_scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.composer_timeline_scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        self.composer_timeline_scroll.setMinimumHeight(235)
        self.composer_timeline_container = SequenceTimelineContainer(self)
        self.composer_timeline_scroll.setWidget(self.composer_timeline_container)
        chain_layout.addWidget(self.composer_timeline_scroll)
        main_layout.addWidget(source_group, 2)

        composer_ctrl_group = QGroupBox("⚙️ 시퀀스 컨트롤 패널")
        composer_ctrl_group.setStyleSheet("font-weight: bold; font-size: 13pt;")
        controls = QHBoxLayout(composer_ctrl_group)
        self.composer_btn_play = QPushButton("▶️ 재생")
        self.composer_btn_play.clicked.connect(self.play_composed_sequence)
        self.composer_btn_pause = QPushButton("⏸️ 일시정지")
        self.composer_btn_pause.clicked.connect(self.pause_motion_sequence)
        self.composer_btn_pause.setEnabled(False)
        self.composer_btn_stop = QPushButton("⏹️ 정지")
        self.composer_btn_stop.clicked.connect(self.stop_motion_sequence)
        self.composer_btn_stop.setEnabled(False)
        self.composer_btn_sync = QPushButton("🤖 로봇 동기화: OFF")
        self.composer_btn_sync.setCheckable(True)
        self.composer_btn_sync.toggled.connect(self.toggle_composer_robot_sync)
        composer_repeat_label = QLabel("반복:")
        self.spin_composer_repeat = QSpinBox()
        self.spin_composer_repeat.setRange(1, 999)
        self.spin_composer_repeat.setValue(1)
        self.spin_composer_repeat.setSuffix(" 회")
        self.spin_composer_repeat.setMinimumWidth(85)
        composer_speed_label = QLabel("배속:")
        self.spin_composer_speed = QDoubleSpinBox()
        self.spin_composer_speed.setRange(0.1, 5.0)
        self.spin_composer_speed.setDecimals(2)
        self.spin_composer_speed.setSingleStep(0.05)
        self.spin_composer_speed.setValue(1.0)
        self.spin_composer_speed.setSuffix("x")
        self.spin_composer_speed.setMinimumWidth(90)
        default_button = QPushButton("🏠 기본자세 복귀")
        default_button.clicked.connect(self.return_to_default_pose)
        clear_button = QPushButton("🧹 초기화")
        clear_button.clicked.connect(self.clear_sequence_composer)
        save_button = QPushButton("💾 저장")
        save_button.clicked.connect(self.save_composed_sequence)
        manage_button = QPushButton("📁 관리/불러오기")
        manage_button.clicked.connect(self.open_sequence_manager)
        export_button = QPushButton("🚀 Jetson 내보내기")
        export_button.clicked.connect(self.export_composed_sequence_json)
        for button in (
            self.composer_btn_play, self.composer_btn_pause, self.composer_btn_stop,
            self.composer_btn_sync,
        ):
            button.setMinimumHeight(45)
            controls.addWidget(button)
        controls.addWidget(composer_repeat_label)
        controls.addWidget(self.spin_composer_repeat)
        controls.addWidget(composer_speed_label)
        controls.addWidget(self.spin_composer_speed)
        for button in (default_button, clear_button, save_button, manage_button, export_button):
            button.setMinimumHeight(45)
            controls.addWidget(button)
        main_layout.addWidget(composer_ctrl_group, 1)
        main_layout.addWidget(chain_group, 3)

    def refresh_composer_source_list(self):
        if not hasattr(self, 'composer_source_list'):
            return
        self.composer_source_list.clear()
        for idx, sequence in enumerate(self.saved_sequences):
            frames = sequence.get('frames', [])
            duration = max(
                (frame.get('start_ms', 0) + frame.get('time_ms', 0) for frame in frames),
                default=0,
            )
            item = QListWidgetItem(
                f"[{idx + 1}] {sequence['name']} "
                f"({len(frames)} 프레임 / {duration}ms / "
                f"{sequence.get('repeat_count', 1)}회 / {sequence.get('playback_speed', 1.0):.2f}x)"
            )
            item.setData(Qt.UserRole, idx)
            self.composer_source_list.addItem(item)
        if hasattr(self, 'sequence_composer_entries'):
            for entry in self.sequence_composer_entries:
                idx = entry['sequence_idx']
                if 0 <= idx < len(self.saved_sequences):
                    sequence = self.saved_sequences[idx]
                    entry['name'] = sequence['name']
                    entry['time_ms'] = self.sequence_duration(idx)
                    entry['frame_count'] = len(sequence.get('frames', []))
                    entry['leading_wait_ms'] = self.sequence_leading_wait(idx)
                    entry['frames'] = self.sequence_frame_summaries(idx)
            self.pack_sequence_composer_entries()
            self.refresh_sequence_composer_timeline()

    def sequence_duration(self, sequence_idx):
        frames = self.saved_sequences[sequence_idx].get('frames', [])
        if not frames:
            return 0
        # 저장 시퀀스의 0ms부터 첫 프레임까지도 의도적인 대기시간입니다.
        # 조합 과정에서 최소 start_ms를 빼면 이 대기가 사라지므로 전체
        # 타임라인 종료시각을 그대로 시퀀스 길이로 사용합니다.
        return max(
            0,
            max(
                frame.get('start_ms', 0) + frame.get('time_ms', 0)
                for frame in frames
            ),
        )

    def sequence_leading_wait(self, sequence_idx):
        frames = self.saved_sequences[sequence_idx].get('frames', [])
        if not frames:
            return 0
        return max(0, min(int(frame.get('start_ms', 0)) for frame in frames))

    def sequence_frame_summaries(self, sequence_idx):
        frames = self.saved_sequences[sequence_idx].get('frames', [])
        return [
            {
                "name": str(frame.get("name", "Frame")),
                "start_ms": max(0, int(frame.get("start_ms", 0))),
                "time_ms": max(
                    MIN_TIMELINE_FRAME_MS,
                    int(frame.get("time_ms", MIN_TIMELINE_FRAME_MS)),
                ),
            }
            for frame in sorted(frames, key=lambda item: item.get("start_ms", 0))
        ]

    def add_sequence_to_composer(self, sequence_idx=None, drop_x=None):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        # 직접 슬롯으로 연결된 과거 호출도 clicked(bool)의 False를 0번
        # 시퀀스로 해석하지 않도록 방어합니다.
        if isinstance(sequence_idx, bool):
            sequence_idx = None
        if sequence_idx is None:
            item = self.composer_source_list.currentItem()
            if item is None:
                return QMessageBox.warning(self, "경고", "추가할 저장 시퀀스를 선택하세요.")
            sequence_idx = item.data(Qt.UserRole)
        sequence = self.saved_sequences[sequence_idx]
        duration = self.sequence_duration(sequence_idx)
        entry = {
            "sequence_idx": sequence_idx,
            "name": sequence['name'],
            "time_ms": duration,
            "frame_count": len(sequence.get('frames', [])),
            "leading_wait_ms": self.sequence_leading_wait(sequence_idx),
            "frames": self.sequence_frame_summaries(sequence_idx),
            "start_ms": self.composer_total_duration(),
        }
        self.sequence_composer_entries.append(entry)
        if drop_x is not None:
            self.reorder_sequence_composer_entry(entry, drop_x)
        else:
            self.pack_sequence_composer_entries()
            self.refresh_sequence_composer_timeline()

    def remove_sequence_composer_entry(self, index):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        if 0 <= index < len(self.sequence_composer_entries):
            self.sequence_composer_entries.pop(index)
            self.pack_sequence_composer_entries()
            self.refresh_sequence_composer_timeline()

    def clear_sequence_composer(self):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        self.sequence_composer_entries.clear()
        self.refresh_sequence_composer_timeline()

    def composer_sequence_indices(self):
        return [entry['sequence_idx'] for entry in self.sequence_composer_entries]

    def composer_total_duration(self):
        return sum(entry['time_ms'] for entry in self.sequence_composer_entries)

    def composer_timeline_width(self):
        return max(1200, int(self.composer_total_duration() * self.COMPOSER_SCALE) + 10)

    def pack_sequence_composer_entries(self):
        current = 0
        for entry in self.sequence_composer_entries:
            entry['start_ms'] = current
            current += entry['time_ms']

    def reorder_sequence_composer_entry(self, moved_entry, moved_x):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        others = [entry for entry in self.sequence_composer_entries if entry is not moved_entry]
        moved_center = moved_x + moved_entry['time_ms'] * self.COMPOSER_SCALE / 2.0
        insert_at = 0
        for entry in others:
            center = (entry['start_ms'] + entry['time_ms'] / 2.0) * self.COMPOSER_SCALE
            if moved_center >= center:
                insert_at += 1
        others.insert(insert_at, moved_entry)
        self.sequence_composer_entries = others
        self.pack_sequence_composer_entries()
        self.refresh_sequence_composer_timeline()

    def refresh_sequence_composer_timeline(self):
        if not hasattr(self, 'composer_timeline_container'):
            return
        for child in self.composer_timeline_container.findChildren(SequenceTimelineBlockWidget):
            child.setParent(None)
            child.deleteLater()
        for index, entry in enumerate(self.sequence_composer_entries):
            block = SequenceTimelineBlockWidget(entry, index, self)
            block.setParent(self.composer_timeline_container)
            x = int(entry['start_ms'] * self.COMPOSER_SCALE)
            width = max(1, int(entry['time_ms'] * self.COMPOSER_SCALE))
            block.setGeometry(x, 40, width, 145)
            block.show()
        self.composer_timeline_container.setMinimumSize(
            self.composer_timeline_width(), 205
        )
        self.composer_timeline_container.update()
        self.refresh_composer_meta()

    def build_composed_sequence_frames(self):
        combined_frames = []
        current_start = 0
        for sequence_idx in self.composer_sequence_indices():
            source_frames = self.saved_sequences[sequence_idx].get('frames', [])
            if not source_frames:
                continue
            ordered = sorted(source_frames, key=lambda frame: frame.get('start_ms', 0))
            source_end = max(
                frame.get('start_ms', 0) + max(MIN_TIMELINE_FRAME_MS, frame.get('time_ms', 0))
                for frame in ordered
            )
            for source_frame in ordered:
                new_frame = copy.deepcopy(source_frame)
                new_frame['time_ms'] = max(MIN_TIMELINE_FRAME_MS, new_frame['time_ms'])
                # 각 원본 시퀀스의 0ms 기준을 그대로 붙여 맨 앞 대기와
                # 내부 프레임 사이 대기시간을 모두 보존합니다.
                new_frame['start_ms'] = (
                    current_start + max(0, source_frame.get('start_ms', 0))
                )
                combined_frames.append(new_frame)
            current_start += source_end
        return combined_frames, current_start

    def refresh_composer_meta(self, *args):
        if not hasattr(self, 'lbl_composer_meta'):
            return
        frames, duration = self.build_composed_sequence_frames()
        self.lbl_composer_meta.setText(
            f"구성: {len(self.sequence_composer_entries)}개 시퀀스 / {len(frames)}개 프레임 / {duration}ms"
        )

    def save_composed_sequence(self):
        if not self.sequence_composer_entries:
            return QMessageBox.warning(self, "경고", "먼저 조합할 시퀀스를 추가하세요.")
        frames, duration = self.build_composed_sequence_frames()
        if not frames:
            return QMessageBox.warning(self, "경고", "조합 결과에 프레임이 없습니다.")
        if duration > 60000:
            return QMessageBox.warning(self, "시간 초과", "조합 결과가 최대 타임라인 길이 60000ms를 초과합니다.")

        name, ok = QInputDialog.getText(self, "시퀀스 조합 저장", "새 시퀀스 이름:")
        if not ok or not name.strip():
            return
        # 조합 결과를 지금 새 시퀀스로 저장하는 경우부터 새 궤적 방식을
        # 사용합니다. 조합에 사용된 기존 저장 시퀀스 원본은 변경하지 않습니다.
        for frame in frames:
            frame["lift_early_arrival"] = True
        self.saved_sequences.append({
            "name": name.strip(),
            "max_seq_ms": max(1000, duration),
            "repeat_count": self.spin_composer_repeat.value(),
            "playback_speed": self.spin_composer_speed.value(),
            "frames": frames,
        })
        self.refresh_sequence_list()
        self.save_persistent_state()
        QMessageBox.information(
            self,
            "저장 완료",
            f"'{name.strip()}' 시퀀스를 {len(frames)}개 프레임, {duration}ms로 저장했습니다.",
        )

    def activate_composer_playback(self):
        frames, duration = self.build_composed_sequence_frames()
        if not frames:
            QMessageBox.warning(self, "경고", "재생할 조합 시퀀스가 없습니다.")
            return False
        if self.playback_context != "composer":
            self.composer_motion_backup = (self.motion_sequence, self.max_seq_ms)
        self.motion_sequence = copy.deepcopy(frames)
        self.max_seq_ms = max(1, duration)
        self.playback_context = "composer"
        return True

    def play_composed_sequence(self):
        if self.playback_context != "composer" and not self.activate_composer_playback():
            return
        if not self.is_paused:
            self.playback_repeat_target = self.spin_composer_repeat.value()
            self.playback_repeat_current = 1
            self.playback_speed = self.spin_composer_speed.value()
        self.play_keyframe_sequence()

    def toggle_composer_robot_sync(self, checked):
        if checked:
            if not self.sequence_composer_entries:
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(False)
                self.composer_btn_sync.blockSignals(False)
                return QMessageBox.warning(self, "경고", "먼저 조합할 시퀀스를 추가하세요.")
            if self.robot_sync_enabled:
                self.composer_btn_sync.setText("🤖 로봇 동기화: ON")
                return
            original_motion, original_max = self.motion_sequence, self.max_seq_ms
            frames, duration = self.build_composed_sequence_frames()
            self.motion_sequence, self.max_seq_ms = frames, max(1, duration)
            self.btn_robot_sync.setChecked(True)
            self.motion_sequence, self.max_seq_ms = original_motion, original_max
            if not self.robot_sync_enabled:
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(False)
                self.composer_btn_sync.blockSignals(False)
            else:
                self.composer_btn_sync.setText("🤖 로봇 동기화: ON")
        else:
            if self.robot_sync_enabled:
                self.btn_robot_sync.setChecked(False)
            self.composer_btn_sync.setText("🤖 로봇 동기화: OFF")

    def export_composed_sequence_json(self):
        frames, duration = self.build_composed_sequence_frames()
        if not frames:
            return QMessageBox.warning(self, "경고", "내보낼 조합 시퀀스가 없습니다.")
        file_name, _ = QFileDialog.getSaveFileName(
            self, "조합 시퀀스 저장", "jetson_composed_sequence.json", "JSON Files (*.json)"
        )
        if file_name:
            with open(file_name, 'w', encoding='utf-8') as file:
                json.dump({
                    "max_seq_ms": duration,
                    "repeat_count": self.spin_composer_repeat.value(),
                    "playback_speed": self.spin_composer_speed.value(),
                    "frames": frames,
                }, file, indent=4, ensure_ascii=False)
            QMessageBox.information(self, "성공", "조합 시퀀스 데이터 추출 완료!")

    def apply_timeline_zoom(self, checked=False):
        value = self.spin_timeline_zoom.value()
        old_scale = self.SCALE
        viewport_center_ms = (
            self.timeline_scroll.horizontalScrollBar().value()
            + self.timeline_scroll.viewport().width() / 2
        ) / old_scale
        self.SCALE = value / 100.0
        self.refresh_timeline_ui()
        new_scroll = int(viewport_center_ms * self.SCALE - self.timeline_scroll.viewport().width() / 2)
        self.timeline_scroll.horizontalScrollBar().setValue(max(0, new_scroll))

    def auto_fit_timeline_to_frame_times(self):
        """프레임 길이에 맞춰 타임라인 표시 배율만 자동 조절합니다."""
        if not self.motion_sequence:
            return
        durations = sorted(
            max(MIN_TIMELINE_FRAME_MS, int(frame.get("time_ms", 0)))
            for frame in self.motion_sequence
        )
        middle = len(durations) // 2
        if len(durations) % 2:
            median_ms = durations[middle]
        else:
            median_ms = (durations[middle - 1] + durations[middle]) / 2.0

        # 걷기 프레임은 50ms 눈금이 보이도록 크게, 허들처럼
        # 긴 프레임은 200~500ms 눈금이 보이도록 축소합니다.
        if median_ms <= 150:
            zoom_percent = 250
        elif median_ms <= 300:
            zoom_percent = 150
        elif median_ms <= 600:
            zoom_percent = 75
        elif median_ms <= 1000:
            zoom_percent = 50
        else:
            zoom_percent = 25

        self.spin_timeline_zoom.setValue(zoom_percent)
        self.apply_timeline_zoom()
        print(
            f"[🔍 타임라인 자동 맞춤] "
            f"프레임 중앙값={median_ms:.0f}ms -> {zoom_percent}%"
        )

    def apply_composer_timeline_zoom(self, value):
        old_scale = self.COMPOSER_SCALE
        center_ms = (
            self.composer_timeline_scroll.horizontalScrollBar().value()
            + self.composer_timeline_scroll.viewport().width() / 2
        ) / old_scale
        self.COMPOSER_SCALE = value / 100.0
        self.refresh_sequence_composer_timeline()
        new_scroll = int(
            center_ms * self.COMPOSER_SCALE
            - self.composer_timeline_scroll.viewport().width() / 2
        )
        self.composer_timeline_scroll.horizontalScrollBar().setValue(max(0, new_scroll))

    def previous_frame_end_ms(self, target_frame):
        ordered = sorted(self.motion_sequence, key=lambda frame: frame['start_ms'])
        index = next(
            (idx for idx, frame in enumerate(ordered) if frame is target_frame),
            -1,
        )
        if index <= 0:
            return 0
        previous = ordered[index - 1]
        return previous['start_ms'] + previous['time_ms']

    def frame_gap_ms(self, target_frame):
        return max(0, target_frame['start_ms'] - self.previous_frame_end_ms(target_frame))

    @staticmethod
    def shift_following_frames_for_duration(frames, target_frame, new_duration_ms, old_duration_ms=None):
        """프레임 길이 변화량만큼 뒤 프레임 전체를 이동해 기존 간격을 보존합니다."""
        ordered = sorted(frames, key=lambda frame: frame['start_ms'])
        index = next(
            (idx for idx, frame in enumerate(ordered) if frame is target_frame),
            -1,
        )
        if index < 0:
            return False, 0, 0

        old_duration_ms = int(
            target_frame.get('time_ms', MIN_TIMELINE_FRAME_MS)
            if old_duration_ms is None else old_duration_ms
        )
        new_duration_ms = max(MIN_TIMELINE_FRAME_MS, int(new_duration_ms))
        delta_ms = new_duration_ms - old_duration_ms

        proposed_end_ms = target_frame['start_ms'] + new_duration_ms
        for frame in ordered[index + 1:]:
            proposed_end_ms = max(
                proposed_end_ms,
                frame['start_ms'] + delta_ms + frame['time_ms'],
            )
        if proposed_end_ms > 60000:
            return False, delta_ms, proposed_end_ms

        target_frame['time_ms'] = new_duration_ms
        if delta_ms:
            for frame in ordered[index + 1:]:
                frame['start_ms'] += delta_ms
        return True, delta_ms, proposed_end_ms

    def change_motion_frame_duration(self, target_frame, new_duration_ms, old_duration_ms=None):
        """타임라인 프레임 시간 변경을 적용하고 뒤 프레임을 함께 이동합니다."""
        changed, delta_ms, final_end_ms = self.shift_following_frames_for_duration(
            self.motion_sequence,
            target_frame,
            new_duration_ms,
            old_duration_ms=old_duration_ms,
        )
        if not changed:
            return False

        if final_end_ms > self.max_seq_ms:
            self.max_seq_ms = final_end_ms
            self.spin_max_time.setValue(final_end_ms)
        self.motion_sequence.sort(key=lambda frame: frame['start_ms'])
        if delta_ms:
            print(
                f"[↔️ 프레임 시간 변경] '{target_frame.get('name', 'Frame')}' "
                f"{delta_ms:+d}ms -> 뒤 프레임 전체 동일 이동"
            )
        return True

    def reorder_motion_frame(self, moved_frame, moved_x):
        """빈 위치는 유지하고, 충돌 위치 삽입 시 뒤 블록만 필요한 만큼 밉니다."""
        duration = max(MIN_TIMELINE_FRAME_MS, int(moved_frame['time_ms']))
        max_start = max(0, self.max_seq_ms - duration)
        desired_start = max(0, min(int(round(moved_x / self.SCALE)), max_start))
        others = [frame for frame in self.motion_sequence if frame is not moved_frame]
        original_positions = [(frame, frame['start_ms']) for frame in self.motion_sequence]

        def is_free(start_ms):
            end_ms = start_ms + duration
            return all(
                end_ms <= frame['start_ms']
                or start_ms >= frame['start_ms'] + frame['time_ms']
                for frame in others
            )

        if is_free(desired_start):
            chosen_start = desired_start
        else:
            ordered = sorted(others, key=lambda frame: frame['start_ms'])
            moved_center = desired_start + duration / 2.0
            insert_at = sum(
                moved_center >= frame['start_ms'] + frame['time_ms'] / 2.0
                for frame in ordered
            )
            previous_end = 0
            if insert_at > 0:
                previous = ordered[insert_at - 1]
                previous_end = previous['start_ms'] + previous['time_ms']
            chosen_start = max(desired_start, previous_end)
            ordered.insert(insert_at, moved_frame)
            moved_frame['start_ms'] = chosen_start

            # 삽입 지점 뒤쪽만 밀어 기존의 다른 빈 간격은 가능한 한 유지합니다.
            current_end = chosen_start + duration
            for frame in ordered[insert_at + 1:]:
                if frame['start_ms'] < current_end:
                    frame['start_ms'] = current_end
                current_end = frame['start_ms'] + frame['time_ms']

            if current_end > 60000:
                for frame, original_start in original_positions:
                    frame['start_ms'] = original_start
                drag_original = moved_frame.get('_drag_original_start_ms')
                if drag_original is None:
                    self.motion_sequence.remove(moved_frame)
                else:
                    moved_frame['start_ms'] = drag_original
                    moved_frame.pop('_drag_original_start_ms', None)
                return False
            if current_end > self.max_seq_ms:
                self.max_seq_ms = current_end
                self.spin_max_time.setValue(current_end)

        moved_frame['start_ms'] = chosen_start
        moved_frame.pop('_drag_original_start_ms', None)
        self.motion_sequence.sort(key=lambda frame: frame['start_ms'])
        return True

    def resort_motion_sequence(self):
        if not self.motion_sequence: return
        self.motion_sequence.sort(key=lambda f: f['start_ms'])
        current_end = 0
        for f in self.motion_sequence:
            if f['start_ms'] < current_end:
                f['start_ms'] = current_end
            current_end = f['start_ms'] + f['time_ms']
            
        excess = current_end - self.max_seq_ms
        if excess > 0:
            limit_end = self.max_seq_ms
            for f in reversed(self.motion_sequence):
                if f['start_ms'] + f['time_ms'] > limit_end:
                    f['start_ms'] = limit_end - f['time_ms']
                limit_end = f['start_ms'] 
                
            if self.motion_sequence[0]['start_ms'] < 0:
                offset = 0 - self.motion_sequence[0]['start_ms']
                for f in self.motion_sequence:
                    f['start_ms'] += offset

    def apply_max_sequence_time(self):
        new_max = self.spin_max_time.value()
        total_len = sum(f['time_ms'] for f in self.motion_sequence)
        if total_len > new_max:
            QMessageBox.warning(self, "공간 부족", f"현재 블록들의 총 길이({total_len}ms)가 설정하려는 도화지 길이({new_max}ms)보다 큽니다!\n블록 길이를 줄인 후 시도하세요.")
            self.spin_max_time.setValue(self.max_seq_ms) 
            return
        self.max_seq_ms = new_max
        self.resort_motion_sequence() 
        self.refresh_timeline_ui()
        QMessageBox.information(self, "완료", f"타임라인 총 길이가 {new_max}ms로 설정되었습니다.")

    def apply_target_time(self):
        if not self.motion_sequence: return
        target_total = self.spin_target_time.value()
        current_total = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)
        if current_total == 0: return
        
        ratio = target_total / current_total
        current_x = 0
        for f in sorted(self.motion_sequence, key=lambda x: x['start_ms']):
            new_start = int(f['start_ms'] * ratio)
            new_time = max(MIN_TIMELINE_FRAME_MS, int(f['time_ms'] * ratio))
            if new_start < current_x: new_start = current_x
            f['start_ms'] = new_start
            f['time_ms'] = new_time
            current_x = new_start + new_time
            
        if current_x > self.max_seq_ms:
            self.max_seq_ms = current_x
            self.spin_max_time.setValue(self.max_seq_ms)
            
        self.refresh_timeline_ui()
        QMessageBox.information(self, "완료", f"전체 시퀀스가 비율 스케일링 되었습니다.")

    def apply_uniform_frame_gap(self):
        if not self.motion_sequence:
            return QMessageBox.warning(self, "경고", "간격을 적용할 프레임이 없습니다.")

        gap_ms = self.spin_uniform_frame_gap.value()
        ordered = sorted(self.motion_sequence, key=lambda frame: frame['start_ms'])
        first_start = max(0, ordered[0]['start_ms'])
        planned_starts = []
        current_start = first_start
        for frame in ordered:
            planned_starts.append(current_start)
            current_start += frame['time_ms'] + gap_ms
        final_end = current_start - gap_ms

        if final_end > 60000:
            return QMessageBox.warning(
                self,
                "시간 초과",
                f"{gap_ms}ms 간격을 적용하면 종료 지점이 {final_end}ms가 되어 "
                "최대 길이 60000ms를 초과합니다.",
            )

        for frame, start_ms in zip(ordered, planned_starts):
            frame['start_ms'] = start_ms
        self.motion_sequence = ordered
        if final_end > self.max_seq_ms:
            self.max_seq_ms = final_end
            self.spin_max_time.setValue(final_end)
        self.refresh_timeline_ui()
        QMessageBox.information(
            self,
            "간격 적용 완료",
            f"모든 프레임 사이 간격을 {gap_ms}ms로 맞췄습니다.\n"
            f"첫 프레임 시작: {first_start}ms / 종료 지점: {final_end}ms",
        )

    def refresh_timeline_ui(self):
        # 로드된 예전 데이터나 일괄 시간 변경 결과도 그리기 전에 정렬하여
        # 블록이 같은 트랙에서 서로 겹치는 상태를 허용하지 않습니다.
        self.resort_motion_sequence()
        for child in self.timeline_container.findChildren(TimelineBlockWidget):
            child.setParent(None)
            child.deleteLater()
            
        for idx, frame_data in enumerate(self.motion_sequence):
            block = TimelineBlockWidget(frame_data, idx, self)
            block.setParent(self.timeline_container)
            x_pos = int(frame_data['start_ms'] * self.SCALE)
            w = int(frame_data['time_ms'] * self.SCALE) 
            # 편집 위젯은 연결선 위쪽에 두고, 아래쪽에는 비아포인트 선과
            # 각 프레임의 도착시각을 표시합니다.
            block.setGeometry(x_pos, 40, w, 105)
            block.show()
            
        fixed_width = int(self.max_seq_ms * self.SCALE)
        self.timeline_container.setMinimumSize(fixed_width + 10, 250)
        if hasattr(self, 'spin_uniform_frame_gap'):
            ordered = sorted(self.motion_sequence, key=lambda frame: frame['start_ms'])
            gaps = [
                ordered[index]['start_ms']
                - (ordered[index - 1]['start_ms'] + ordered[index - 1]['time_ms'])
                for index in range(1, len(ordered))
            ]
            displayed_gap = gaps[0] if gaps and all(gap == gaps[0] for gap in gaps) else 0
            if 0 <= displayed_gap <= self.spin_uniform_frame_gap.maximum():
                self.spin_uniform_frame_gap.blockSignals(True)
                self.spin_uniform_frame_gap.setValue(displayed_gap)
                self.spin_uniform_frame_gap.blockSignals(False)
        self.refresh_timeline_meta()
        self.save_persistent_state()

    def refresh_timeline_meta(self):
        if not self.motion_sequence: 
            self.lbl_total_time.setText(f"현재 모션 종료 지점: 0ms / 타임라인 길이: {self.max_seq_ms}ms")
        else:
            current_end = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)
            self.lbl_total_time.setText(f"현재 모션 종료 지점: {current_end}ms / 타임라인 길이: {self.max_seq_ms}ms")
        self.timeline_container.update()
        if hasattr(self, "trajectory_graph"):
            self.trajectory_graph.update()

    def motion_end_ms(self):
        return max(
            (
                int(frame.get("start_ms", 0))
                + int(frame.get("time_ms", 0))
                for frame in self.motion_sequence
            ),
            default=0,
        )

    def add_to_motion(self):
        selected_items = self.frame_list_all.selectedItems()
        if not selected_items:
            return QMessageBox.warning(self, "경고", "추가할 프레임을 선택하세요.")

        current_end_ms = 0
        if self.motion_sequence: current_end_ms = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)

        for item in selected_items:
            original_idx = item.data(Qt.UserRole)
            new_frame = copy.deepcopy(self.frames[original_idx])
            new_frame["source_frame_id"] = new_frame.get("frame_id")
            new_frame["lift_early_arrival"] = True
            new_frame['time_ms'] = DEFAULT_SEQUENCE_FRAME_MS
            
            if current_end_ms + new_frame['time_ms'] > self.max_seq_ms:
                QMessageBox.warning(self, "공간 부족", f"타임라인 공간이 부족하여 추가할 수 없습니다.\n여백을 확보하거나 총 길이를 늘려주세요.")
                break 
            
            new_frame['start_ms'] = current_end_ms
            self.motion_sequence.append(new_frame)
            current_end_ms = new_frame['start_ms'] + new_frame['time_ms']
            
        self.resort_motion_sequence()
        self.refresh_timeline_ui()

    def remove_from_motion_by_idx(self, idx):
        if 0 <= idx < len(self.motion_sequence):
            self.motion_sequence.pop(idx)
            self.resort_motion_sequence()
            self.refresh_timeline_ui()

    def clear_motion(self):
        if not self.motion_sequence: return
        if QMessageBox.question(self, '확인', "모두 비우시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.motion_sequence.clear()
            self.loaded_sequence_id = None
            self.lbl_loaded_sequence.setText("현재 타임라인: 새 시퀀스")
            self.update_sequence_save_button()
            self.refresh_timeline_ui()

    def save_sequence(self):
        if not self.motion_sequence: return QMessageBox.warning(self, "경고", "저장할 타임라인이 비어있습니다.")
        self.repair_legacy_sequence_timing(self.motion_sequence)
        seq_name, ok = QInputDialog.getText(self, '저장', '새로운 시퀀스 이름 입력:')
        if ok and seq_name.strip():
            saved_frames = copy.deepcopy(self.motion_sequence)
            for frame in saved_frames:
                frame["lift_early_arrival"] = True
            self.saved_sequences.append({
                "sequence_id": uuid.uuid4().hex,
                "name": seq_name.strip(), 
                "max_seq_ms": self.max_seq_ms,
                "repeat_count": self.spin_motion_repeat.value(),
                "playback_speed": self.spin_motion_speed.value(),
                "repeatable": True,
                "completion": {
                    "position_tolerance_deg": 2.0,
                    "settle_duration_ms": 80,
                    "settle_timeout_ms": 3000,
                },
                "frames": saved_frames
            })
            self.loaded_sequence_id = self.saved_sequences[-1]["sequence_id"]
            self.lbl_loaded_sequence.setText(
                f"현재 타임라인: {seq_name.strip()} (저장됨)"
            )
            self.update_sequence_save_button()
            self.refresh_sequence_list()
            self.save_persistent_state()
            QMessageBox.information(self, "저장 완료", f"'{seq_name.strip()}' 시퀀스 저장됨.")

    def load_saved_sequence(self, idx):
        if idx < 0 or idx >= len(self.saved_sequences):
            return
        target_seq = self.saved_sequences[idx]
        target_seq.setdefault("sequence_id", uuid.uuid4().hex)
        self.loaded_sequence_id = target_seq["sequence_id"]
        self.max_seq_ms = target_seq.get("max_seq_ms", 5000)
        self.spin_max_time.setValue(self.max_seq_ms)
        self.spin_motion_repeat.setValue(max(1, int(target_seq.get("repeat_count", 1))))
        self.spin_motion_speed.setValue(
            max(0.1, min(5.0, float(target_seq.get("playback_speed", 1.0))))
        )
        self.motion_sequence = copy.deepcopy(target_seq["frames"])
        self.refresh_timeline_ui()
        self.auto_fit_timeline_to_frame_times()
        self.lbl_loaded_sequence.setText(f"현재 타임라인: {target_seq['name']} (불러옴)")
        self.update_sequence_save_button()
        self.tabs.setCurrentWidget(self.tab_motion)

    def loaded_sequence_index(self):
        if not self.loaded_sequence_id:
            return -1
        return next(
            (
                index for index, sequence in enumerate(self.saved_sequences)
                if sequence.get("sequence_id") == self.loaded_sequence_id
            ),
            -1,
        )

    def update_sequence_save_button(self):
        if not hasattr(self, "btn_update_sequence"):
            return
        index = self.loaded_sequence_index()
        enabled = index >= 0
        self.btn_update_sequence.setEnabled(enabled)
        if enabled:
            self.btn_update_sequence.setText(
                f"♻️ '{self.saved_sequences[index]['name']}' 재저장"
            )
        else:
            self.btn_update_sequence.setText("♻️ 현재 시퀀스 재저장")

    def refresh_loaded_sequence_indicator(self):
        index = self.loaded_sequence_index()
        if index >= 0 and hasattr(self, "lbl_loaded_sequence"):
            self.lbl_loaded_sequence.setText(
                f"현재 타임라인: {self.saved_sequences[index]['name']} (불러옴)"
            )
        elif hasattr(self, "lbl_loaded_sequence"):
            self.loaded_sequence_id = None
            self.lbl_loaded_sequence.setText("현재 타임라인: 새 시퀀스")
        self.update_sequence_save_button()

    def update_loaded_sequence(self):
        if not self.motion_sequence:
            return QMessageBox.warning(self, "경고", "재저장할 타임라인이 비어있습니다.")
        index = self.loaded_sequence_index()
        if index < 0:
            self.loaded_sequence_id = None
            self.update_sequence_save_button()
            return QMessageBox.warning(
                self,
                "재저장 불가",
                "불러온 원본 시퀀스가 없습니다. 먼저 저장 시퀀스를 불러오거나 새로 저장하세요.",
            )
        sequence = self.saved_sequences[index]
        name = sequence["name"]
        reply = QMessageBox.question(
            self,
            "시퀀스 재저장",
            f"'{name}' 시퀀스를 현재 타임라인 내용으로 덮어쓰시겠습니까?\n"
            "프레임 시간, 반복 횟수, 재생속도가 함께 저장됩니다.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return
        self.repair_legacy_sequence_timing(self.motion_sequence)
        sequence.update({
            "max_seq_ms": self.max_seq_ms,
            "repeat_count": self.spin_motion_repeat.value(),
            "playback_speed": self.spin_motion_speed.value(),
            "frames": copy.deepcopy(self.motion_sequence),
        })
        self.refresh_sequence_list()
        self.save_persistent_state()
        self.lbl_loaded_sequence.setText(f"현재 타임라인: {name} (재저장됨)")
        self.update_sequence_save_button()
        QMessageBox.information(
            self,
            "재저장 완료",
            f"'{name}' 시퀀스를 현재 타임라인 내용으로 재저장했습니다.\n"
            "잘못 저장한 경우 Ctrl+Z로 복구할 수 있습니다.",
        )

    def load_selected_sequence_to_timeline(self):
        item = self.sequence_list_ui.currentItem()
        if item is None:
            return QMessageBox.warning(self, "경고", "불러올 저장 시퀀스를 선택하세요.")
        self.load_saved_sequence(item.data(Qt.UserRole))

    def load_sequence_from_list_item(self, item):
        idx = item.data(Qt.UserRole)
        if idx is not None:
            self.load_saved_sequence(idx)

    def open_sequence_manager(self):
        if hasattr(self, 'sequence_list_ui') and self.sequence_list_ui.currentItem():
            idx = self.sequence_list_ui.currentItem().data(Qt.UserRole)
            if idx is not None:
                self.load_saved_sequence(idx)
                return
        if not self.saved_sequences: return QMessageBox.information(self, "안내", "저장된 시퀀스가 없습니다.")
        items = [f"[{i+1}] {seq['name']} ({len(seq['frames'])} 프레임)" for i, seq in enumerate(self.saved_sequences)]
        item, ok = QInputDialog.getItem(self, "불러오기", "시퀀스 선택 (현재 덮어씌워짐):", items, 0, False)
        if ok and item:
            idx = items.index(item)
            self.load_saved_sequence(idx)

    def update_playback_buttons(self):
        self.btn_keyframe_play.setEnabled(not self.is_playing)
        self.spin_motion_repeat.setEnabled(
            not self.is_playing and not (self.is_paused and self.playback_context == "motion")
        )
        self.spin_motion_speed.setEnabled(
            not self.is_playing and not (self.is_paused and self.playback_context == "motion")
        )
        self.btn_keyframe_pause.setEnabled(self.is_playing)
        self.btn_stop_motion.setEnabled(self.is_playing or self.is_paused or self.current_timeline_ms > 0)
        if self.is_playing and self.playback_context == "motion":
            self.btn_keyframe_play.setText(
                f"▶️ 재생 중 ({self.playback_repeat_current}/{self.playback_repeat_target}, "
                f"{self.playback_speed:.2f}x)"
            )
        else:
            self.btn_keyframe_play.setText("▶️ 계속 재생" if self.is_paused else "▶️ 재생")
        if hasattr(self, 'composer_btn_play'):
            composer_active = self.playback_context == "composer"
            self.spin_composer_repeat.setEnabled(
                not self.is_playing and not (self.is_paused and composer_active)
            )
            self.spin_composer_speed.setEnabled(
                not self.is_playing and not (self.is_paused and composer_active)
            )
            self.composer_btn_play.setEnabled(not self.is_playing)
            self.composer_btn_pause.setEnabled(self.is_playing and composer_active)
            self.composer_btn_stop.setEnabled(composer_active and (self.is_playing or self.is_paused))
            if self.is_playing and composer_active:
                self.composer_btn_play.setText(
                    f"▶️ 재생 중 ({self.playback_repeat_current}/{self.playback_repeat_target}, "
                    f"{self.playback_speed:.2f}x)"
                )
            else:
                self.composer_btn_play.setText("▶️ 계속 재생" if composer_active and self.is_paused else "▶️ 재생")

    def toggle_robot_sync(self, checked):
        if checked:
            if not self.motion_sequence:
                QMessageBox.warning(self, "경고", "로봇과 동기화할 키프레임 시퀀스가 없습니다.")
                self.btn_robot_sync.blockSignals(True)
                self.btn_robot_sync.setChecked(False)
                self.btn_robot_sync.blockSignals(False)
                return
            reply = QMessageBox.question(
                self,
                "로봇 동기화",
                "⚠️ 동기화를 켜면 재생 및 타임라인 이동에 따라 로봇이 실제로 움직입니다.\n"
                "로봇 주변이 안전한지 확인하셨습니까?",
                QMessageBox.Yes | QMessageBox.No,
                QMessageBox.No,
            )
            if reply != QMessageBox.Yes or not self.prepare_real_robot_sequence():
                self.btn_robot_sync.blockSignals(True)
                self.btn_robot_sync.setChecked(False)
                self.btn_robot_sync.blockSignals(False)
                return
            if not self.sync_sequence_start_from_robot():
                self.btn_robot_sync.blockSignals(True)
                self.btn_robot_sync.setChecked(False)
                self.btn_robot_sync.blockSignals(False)
                return
            self.robot_sync_enabled = True
            self.execute_on_real_robot = True
            self.robot_scrub_command_angles = self.joints.copy()
            self.robot_scrub_target_angles = self.joints.copy()
            self.btn_robot_sync.setText("🤖 로봇 동기화: ON")
            if hasattr(self, 'composer_btn_sync'):
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(True)
                self.composer_btn_sync.setText("🤖 로봇 동기화: ON")
                self.composer_btn_sync.blockSignals(False)
            print("[🔗 로봇 동기화 ON] 재생 및 타임라인 탐색 자세를 실제 로봇에 전송합니다.")
        else:
            if self.is_playing:
                self.hold_timed_motion_at_feedback()
            self.robot_sync_enabled = False
            self.execute_on_real_robot = False
            self.robot_scrub_timer.stop()
            self.robot_scrub_target_angles = {}
            self.robot_scrub_command_angles = {}
            self.btn_robot_sync.setText("🤖 로봇 동기화: OFF")
            if hasattr(self, 'composer_btn_sync'):
                self.composer_btn_sync.blockSignals(True)
                self.composer_btn_sync.setChecked(False)
                self.composer_btn_sync.setText("🤖 로봇 동기화: OFF")
                self.composer_btn_sync.blockSignals(False)
            print("[🔓 로봇 동기화 OFF] 타임라인 조작은 3D 미리보기에만 반영됩니다.")

    def sync_sequence_start_from_robot(self):
        """첫 키프레임 보간 전에 실제 로봇 자세를 시작값으로 확정합니다."""
        target_ids = sorted(self.sequence_joint_ids() & set(self.online_joints))
        actual_angles = {}
        failed_ids = []
        for j_id in target_ids:
            angle = self.read_present_angle(j_id)
            if angle is None:
                failed_ids.append(j_id)
                continue
            actual_angles[j_id] = angle
            self.update_joint_display(j_id, angle, update_robot=False)

        if failed_ids:
            QMessageBox.warning(
                self,
                "통신 에러",
                "시퀀스 시작 자세를 읽지 못해 로봇 동기화를 중단합니다.\n"
                f"ID: {failed_ids}",
            )
            return False

        self.update_3d_robot()
        self.feedback_angles.update(actual_angles)
        self.commanded_angles.update(actual_angles)
        self.commanded_joint_ids.update(actual_angles.keys())
        self.robot_scrub_command_angles = actual_angles.copy()
        self.robot_scrub_target_angles = actual_angles.copy()
        print(f"[✅ 시작 자세 동기화] 실제 로봇 관절 {len(actual_angles)}개의 현재 자세를 사용합니다.")
        return bool(actual_angles)

    def play_keyframe_sequence(self):
        if not self.motion_sequence:
            return QMessageBox.warning(self, "경고", "재생할 시퀀스가 없습니다.")
        end_ms = max(f['start_ms'] + f['time_ms'] for f in self.motion_sequence)
        # 새로 재생할 때는 타임라인을 클릭하거나 이전 재생에서 남은 커서와
        # 관계없이 항상 0ms부터 시작합니다. 일시정지 후 '계속 재생'을 누른
        # 경우에만 멈춘 시각을 유지합니다.
        start_ms = (
            self.current_timeline_ms
            if self.playback_paused_by_button
            and 0 < self.current_timeline_ms < end_ms
            else 0
        )
        self.current_timeline_ms = start_ms
        self.start_motion_playback(
            real_robot=self.robot_sync_enabled,
            start_ms=start_ms,
        )

    def play_motion_page_sequence(self):
        if self.playback_context == "composer":
            self.stop_motion_sequence()
        self.playback_context = "motion"
        if not self.is_paused:
            self.playback_repeat_target = self.spin_motion_repeat.value()
            self.playback_repeat_current = 1
            self.playback_speed = self.spin_motion_speed.value()
        self.play_keyframe_sequence()

    def play_motion_sequence(self, real_robot=False):
        """이전 호출부 호환용 래퍼."""
        if real_robot and not self.robot_sync_enabled:
            self.btn_robot_sync.setChecked(True)
            if not self.robot_sync_enabled:
                return
        self.play_keyframe_sequence()

    def start_motion_playback(self, real_robot=False, start_ms=0):
        # 어떤 경로로 불러온 데이터든 재생 직전에 한 번 더 정렬합니다.
        # 앞 프레임의 종료시각보다 다음 프레임 시작이 빠르면 다음 프레임을
        # 뒤로 밀어 한 시각에 두 프레임 Goal이 겹쳐 전송되지 않게 합니다.
        original_starts = {
            id(frame): int(frame.get("start_ms", 0))
            for frame in self.motion_sequence
        }
        self.resort_motion_sequence()
        overlap_fixed = any(
            int(frame.get("start_ms", 0)) != original_starts[id(frame)]
            for frame in self.motion_sequence
        )
        if overlap_fixed:
            corrected_end_ms = max(
                frame["start_ms"] + frame["time_ms"]
                for frame in self.motion_sequence
            )
            if corrected_end_ms > self.max_seq_ms:
                self.max_seq_ms = corrected_end_ms
                self.spin_max_time.setValue(corrected_end_ms)
            self.refresh_timeline_ui()
            print("[↔️ 프레임 겹침 자동 제거] 앞 프레임 종료 뒤로 순차 배치했습니다.")

        resuming = self.playback_paused_by_button and start_ms > 0
        self.robot_scrub_timer.stop()
        if real_robot and not resuming:
            # 이전 타임라인 클릭에서 남은 robot_scrub_command_angles는
            # 새 재생의 시작 자세가 아닙니다. 재생 직전 실제 Present
            # Position을 다시 읽어 1번 프레임의 보간 시작각으로 사용합니다.
            if not self.sync_sequence_start_from_robot():
                return False
        self.execute_on_real_robot = bool(real_robot and self.robot_sync_enabled)
        self.is_playing = True
        self.is_paused = False
        self.playback_paused_by_button = False
        self.current_timeline_ms = max(0, int(start_ms))
        self.update_playback_buttons()

        self.anim_duration = max(
            f['start_ms'] + f['time_ms'] for f in self.motion_sequence
        ) / 1000.0
        # 첫 프레임의 time_ms도 실제 이동시간입니다. 읽어 둔 실기 시작자세를
        # 첫 목표각으로 덮어쓰면 첫 프레임이 보간되지 않고 즉시 점프하므로,
        # 현재 자세를 그대로 시작점으로 유지합니다.
        if not resuming or not self.start_angles:
            self.start_angles = self.joints.copy()
        self.active_timed_frame_token = None
        self.last_timed_feedback_time = 0.0
        self.timed_profile_error_reported = False
        self.timed_gate_frame = None
        self.timed_gate_wait_started = 0.0
        self.timed_gate_last_check = 0.0
        if self.execute_on_real_robot:
            print(
                "[⏱️ 지정시간 엄수 모드] 200Hz 코사인 궤적 · "
                "프레임 종료시각에 최종 Goal 전송 · 대기시간 없음"
            )
        now = time.perf_counter()
        self.anim_start_time = (
            now - self.current_timeline_ms / (1000.0 * self.playback_speed)
        )
        self.last_playback_ui_time = 0.0
        self.anim_timer.start()

    def pause_motion_sequence(self):
        if not self.is_playing:
            return
        self.current_timeline_ms = max(
            0,
            int((time.perf_counter() - self.anim_start_time) * 1000 * self.playback_speed),
        )
        self.hold_timed_motion_at_feedback()
        self.is_playing = False
        self.is_paused = True
        self.playback_paused_by_button = True
        self.anim_timer.stop()
        self.update_playback_buttons()

    def find_default_pose_row(self):
        configured_row = self.frame_index_from_id(self.default_pose_frame_id)
        if configured_row >= 0:
            return configured_row
        aliases = {"기본자세", "기본", "default", "home", "homepose"}
        for row, frame in enumerate(self.frames):
            normalized_name = "".join(str(frame.get("name", "")).lower().split())
            if normalized_name in aliases:
                return row
        return -1

    def selected_motion_library_frame_index(self):
        item = self.frame_list_all.currentItem()
        if item is None:
            return -1
        frame_idx = item.data(Qt.UserRole)
        if frame_idx is None or not (0 <= frame_idx < len(self.frames)):
            return -1
        return int(frame_idx)

    def load_selected_motion_library_frame(self):
        """모션 제작 탭에서 선택한 라이브러리 자세를 즉시 적용합니다."""
        frame_idx = self.selected_motion_library_frame_index()
        if frame_idx < 0:
            return QMessageBox.warning(
                self, "경고", "전체 프레임 목록에서 불러올 프레임을 선택하세요."
            )
        self.selected_frame_id = self.frames[frame_idx].get("frame_id")
        self.load_frame_to_ui(frame_idx)
        self.apply_selected_frame()

    def set_selected_frame_as_default_pose(self):
        """모션 제작 목록에서 선택한 프레임을 기본자세 버튼에 연결합니다."""
        frame_idx = self.selected_motion_library_frame_index()
        if frame_idx < 0:
            return QMessageBox.warning(
                self, "경고", "전체 프레임 목록에서 기본자세로 사용할 프레임을 선택하세요."
            )
        frame = self.frames[frame_idx]
        self.default_pose_frame_id = frame.get("frame_id")
        self.save_persistent_state()
        QMessageBox.information(
            self,
            "기본자세 설정 완료",
            f"'{frame.get('name', 'Frame')}' 프레임을 기본자세로 설정했습니다.",
        )

    def return_to_default_pose(self):
        """사용자가 버튼을 눌렀을 때만 기본자세 프레임을 저장 시간대로 적용합니다."""
        default_row = self.find_default_pose_row()
        if default_row < 0:
            QMessageBox.warning(
                self,
                "기본자세 없음",
                "저장된 프레임에서 '기본자세'를 찾지 못했습니다.\n"
                "기본으로 사용할 프레임 이름을 '기본자세'로 변경해 주세요.",
            )
            return

        self.selected_frame_id = self.frames[default_row].get("frame_id")
        self.apply_selected_frame()

    def anim_step(self):
        if not self.is_playing: return
        now = time.perf_counter()
        elapsed_sec = now - self.anim_start_time
        t_ms = int(elapsed_sec * 1000 * self.playback_speed)
        if self.execute_on_real_robot or self.robot_sync_enabled:
            t_ms = self.gate_timeline_on_frame_arrival(t_ms, now)
            if t_ms is None:
                return
        self.current_timeline_ms = t_ms

        # 플레이헤드/스타일 갱신은 30fps로 제한해 GUI 렌더링이
        # 200Hz 타임라인 평가와 프레임 경계 전송을 막지 않게 합니다.
        update_visuals = now - self.last_playback_ui_time >= PLAYBACK_UI_INTERVAL_SEC
        if update_visuals:
            self.last_playback_ui_time = now
            if self.playback_context == "composer":
                playhead_px = int(t_ms * self.COMPOSER_SCALE)
                self.composer_timeline_container.set_playhead(True, playhead_px)
                self.composer_timeline_scroll.ensureVisible(
                    playhead_px, self.composer_timeline_scroll.height() // 2, 50, 0
                )
            else:
                playhead_px = int(t_ms * self.SCALE)
                self.timeline_container.set_playhead(True, playhead_px)
                self.timeline_scroll.ensureVisible(playhead_px, self.timeline_scroll.height()//2, 50, 0)
        
        if t_ms >= int(self.anim_duration * 1000):
            end_ms = int(self.anim_duration * 1000)
            last_frame = max(self.motion_sequence, key=lambda f: f['start_ms'] + f['time_ms'])
            # 일반 200Hz tick이 종점 직전에 끝나더라도 마지막 키프레임을
            # 반드시 보간/전송해 종점 명령 누락을 막습니다.
            self.current_timeline_ms = end_ms
            self.scrub_timeline(end_ms, force_robot=True, update_visuals=True)
            if self.playback_repeat_current < self.playback_repeat_target:
                self.playback_repeat_current += 1
                self.current_timeline_ms = 0
                self.anim_start_time = time.perf_counter()
                self.start_angles.update(self.normalize_angles(last_frame.get('angles', {})))
                self.active_timed_frame_token = None
                self.last_timed_feedback_time = 0.0
                if self.playback_context == "composer":
                    self.composer_timeline_container.set_playhead(True, 0)
                    self.composer_timeline_scroll.horizontalScrollBar().setValue(0)
                else:
                    self.timeline_container.set_playhead(True, 0)
                    self.timeline_scroll.horizontalScrollBar().setValue(0)
                self.update_playback_buttons()
                return
            # 마지막 프레임의 시간 프로파일이 종점까지 계속 동작해야 하므로
            # 자연 종료에서는 현재각으로 덮어써 모터를 중간에 멈추지 않습니다.
            self.stop_motion_sequence(keep_timed_goal=True)
            return

        self.scrub_timeline(t_ms, update_visuals=update_visuals)

    def stop_motion_sequence(self, _checked=False, keep_timed_goal=False):
        stopped_context = self.playback_context
        if self.is_playing and not keep_timed_goal:
            self.hold_timed_motion_at_feedback()
        self.is_playing = False
        self.is_paused = False
        self.playback_paused_by_button = False
        self.current_timeline_ms = 0
        self.anim_timer.stop()
        self.robot_scrub_timer.stop()
        self.robot_scrub_target_angles = {}
        self.frame_apply_timer.stop()
        self.frame_apply_ids = []
        self.frame_apply_frame = None
        self.frame_apply_on_complete = None
        self.active_timed_frame_token = None
        self.last_timed_feedback_time = 0.0
        self.timed_gate_frame = None
        self.timed_gate_wait_started = 0.0
        self.timed_gate_last_check = 0.0
        self.execute_on_real_robot = self.robot_sync_enabled
        self.timeline_container.set_playhead(True, 0)
        if hasattr(self, 'composer_timeline_container'):
            self.composer_timeline_container.set_playhead(False, 0)
        if stopped_context == "composer" and self.composer_motion_backup is not None:
            self.motion_sequence, self.max_seq_ms = self.composer_motion_backup
            self.composer_motion_backup = None
            self.playback_context = "motion"
        self.update_playback_buttons()
        for child in self.timeline_container.findChildren(TimelineBlockWidget):
            child.set_default_style()

    def scrub_timeline(self, t_ms, force_robot=False, update_visuals=True):
        self.current_timeline_ms = max(0, min(int(t_ms), self.max_seq_ms))
        if not self.is_playing:
            self.is_paused = self.current_timeline_ms > 0
            self.update_playback_buttons()
        active_frame = None
        last_completed = None
        sequence_end_ms = max(
            (f['start_ms'] + f['time_ms'] for f in self.motion_sequence),
            default=0,
        )

        for f in self.motion_sequence:
            frame_start_ms = f['start_ms']
            frame_end_ms = frame_start_ms + f['time_ms']
            # 인접 프레임 경계는 [start, end)로 판정해야 새 프레임이 정확히
            # 그 경계 tick에서 시작합니다. 시퀀스 최종점만 마지막 프레임에 포함합니다.
            if (frame_start_ms <= t_ms < frame_end_ms
                    or (t_ms == sequence_end_ms == frame_end_ms)):
                active_frame = f
                break
            if frame_end_ms <= t_ms:
                if not last_completed or frame_end_ms > (last_completed['start_ms'] + last_completed['time_ms']):
                    last_completed = f

        if update_visuals:
            idx_to_highlight = self.motion_sequence.index(active_frame) if active_frame else -1
            for child in self.timeline_container.findChildren(TimelineBlockWidget):
                if child.seq_idx == idx_to_highlight: child.set_playing_style()
                else: child.set_default_style()

        target_angles_to_render = {}
        if active_frame:
            # 일부 관절만 저장된 키프레임도 이전 자세 위에 누적합니다.
            # 누락 관절을 0도로 처리하면 시퀀스 시작 시 팔이 펴지는 등
            # 의도하지 않은 중간 자세가 발생합니다.
            base_state = self.start_angles if self.is_playing else self.joints
            prev_state = self.normalize_angles(base_state)
            ordered_frames = sorted(
                self.motion_sequence,
                key=lambda frame: frame['start_ms'] + frame['time_ms'],
            )
            # 첫 실행은 실제 시작 자세에서 1번 프레임으로 갑니다.
            # 반복 2회차부터는 start_angles가 직전 마지막 프레임으로
            # 이미 갱신되므로 별도 강제 처리 없이 자연스럽게 순환합니다.
            for f in ordered_frames:
                end_t = f['start_ms'] + f['time_ms']
                if end_t <= active_frame['start_ms']:
                    prev_state.update(self.normalize_angles(f['angles']))
            
            progress = self.frame_motion_progress(active_frame, t_ms)
            active_angles = self.normalize_angles(active_frame['angles'])
            for j_id in active_angles:
                v0 = prev_state.get(j_id, active_angles[j_id])
                v1 = active_angles.get(j_id, 0)
                target_angles_to_render[j_id] = self.interpolate_angle_shortest(v0, v1, progress)
        else:
            # 프레임이 없는 모든 빈 시간은 같은 '대기' 구간입니다.
            # 시작 자세 위에 프레임을 순서대로 누적합니다. 맨 앞 공백에서는
            # 타임라인의 마지막 프레임 자세를 유지하고, 중간 공백에서는
            # 직전 프레임의 전체 자세를 유지합니다.
            hold_base = (
                self.start_angles
                if self.is_playing and self.start_angles
                else self.joints
            )
            target_angles_to_render = self.normalize_angles(hold_base)
            ordered_frames = sorted(
                self.motion_sequence,
                key=lambda frame: frame['start_ms'] + frame['time_ms'],
            )
            frames_to_hold = []
            for completed_frame in ordered_frames:
                completed_end_ms = (
                    completed_frame['start_ms'] + completed_frame['time_ms']
                )
                if completed_end_ms > t_ms:
                    break
                frames_to_hold.append(completed_frame)

            for completed_frame in frames_to_hold:
                target_angles_to_render.update(
                    self.normalize_angles(completed_frame.get('angles', {}))
                )
            
        if update_visuals:
            self.update_3d_robot(target_angles_to_render)
            if hasattr(self, "trajectory_graph"):
                self.trajectory_graph.update()
        
        if self.is_playing:
            if self.execute_on_real_robot:
                # GUI에서 계산한 half-cosine 궤적의 중간 목표각을 200Hz로
                # 직접 보냅니다. 프레임 최종각만 한 번 보내고 시간이 끝나길
                # 기다리는 방식이 아니므로 화면 궤적과 실기 명령이 같습니다.
                if active_frame is not None:
                    frame_key = active_frame.get("frame_id") or id(active_frame)
                    token = (
                        self.playback_repeat_current,
                        frame_key,
                        int(active_frame.get("start_ms", 0)),
                        int(active_frame.get("time_ms", CONTROL_INTERVAL_MS)),
                    )
                    if token != self.active_timed_frame_token:
                        self.active_timed_frame_token = token
                        self.timed_gate_frame = active_frame
                        self.timed_gate_wait_started = 0.0
                        self.timed_gate_last_check = 0.0
                        print(
                            f"[〰️ GUI 코사인 프레임] "
                            f"'{active_frame.get('name', 'Frame')}' "
                            f"{active_frame.get('time_ms', 0)}ms"
                        )

                target_ids = sorted(
                    set(target_angles_to_render) & set(self.online_joints)
                )
                command_ok = (
                    bool(target_ids)
                    and self.write_goal_positions(
                        target_angles_to_render,
                        target_ids=target_ids,
                        read_feedback=False,
                    )
                )
                if not command_ok:
                    if not self.timed_profile_error_reported:
                        self.timed_profile_error_reported = True
                        QMessageBox.warning(
                            self,
                            "GUI 궤적 전송 실패",
                            "코사인 궤적의 Goal Position을 전송하지 못해 "
                            "실기 재생을 중단합니다.",
                        )
                    self.is_playing = False
                    self.anim_timer.stop()
                    self.update_playback_buttons()
                    return

                self.set_tracking_target_angles(target_angles_to_render)
                now = time.perf_counter()
                if (force_robot
                        or now - self.last_timed_feedback_time >= TIMED_FEEDBACK_INTERVAL_SEC):
                    self.last_timed_feedback_time = now
                    self.read_feedback_and_report_fk()
            elif self.robot_sync_enabled:
                self.queue_robot_scrub_target(target_angles_to_render)
        elif self.robot_sync_enabled:
            self.queue_robot_scrub_target(target_angles_to_render)

    def timeline_state_at(self, t_ms):
        """타임라인 시점의 보간 자세, 저장 토크 상태, 활성 프레임을 반환합니다."""
        t_ms = max(0, min(int(t_ms), self.max_seq_ms))
        base_angles = self.start_angles if self.start_angles else self.joints
        angles = self.normalize_angles(base_angles)
        torque_states = {
            int(j_id): bool(button.isChecked())
            for j_id, button in self.torque_btns.items()
        }
        active_frame = None

        for frame in sorted(self.motion_sequence, key=lambda item: item.get("start_ms", 0)):
            frame_start = int(frame.get("start_ms", 0))
            frame_time = max(MIN_TIMELINE_FRAME_MS, int(frame.get("time_ms", 0)))
            frame_end = frame_start + frame_time
            frame_angles = self.normalize_angles(frame.get("angles", {}))
            frame_torques = {
                int(j_id): bool(value)
                for j_id, value in frame.get("torques", {}).items()
            }

            if t_ms < frame_start:
                break
            if t_ms >= frame_end:
                angles.update(frame_angles)
                torque_states.update(frame_torques)
                continue

            active_frame = frame
            progress = self.frame_motion_progress(frame, t_ms)
            for j_id, target_angle in frame_angles.items():
                start_angle = angles.get(j_id, target_angle)
                angles[j_id] = self.interpolate_angle_shortest(
                    start_angle, target_angle, progress
                )
            torque_states.update(frame_torques)
            break

        return angles, torque_states, active_frame

    def capture_timeline_pose_to_frame_editor(self):
        """현재 비아포인트 사이 자세를 프레임 편집값으로 안전하게 복사합니다."""
        if not self.motion_sequence:
            return QMessageBox.warning(self, "경고", "타임라인에 프레임이 없습니다.")
        if self.is_playing:
            self.pause_motion_sequence()

        capture_ms = self.current_timeline_ms
        angles, torque_states, active_frame = self.timeline_state_at(capture_ms)
        used_feedback = False

        # 로봇이 연결돼 있으면 목표 보간값 대신 실제 도달 자세를 가져옵니다.
        if self.port_opened and self.online_joints:
            if self.read_feedback_and_report_fk():
                for j_id in self.online_joints:
                    if j_id in self.feedback_angles:
                        angles[j_id] = self.feedback_angles[j_id]
                used_feedback = True
            for j_id in self.online_joints:
                actual_torque = self.read_torque_enabled(j_id)
                if actual_torque is not None:
                    torque_states[j_id] = actual_torque

        self.editing_loaded_pose = True
        for j_id, angle in angles.items():
            if j_id in self.sliders:
                self.sliders[j_id].setEnabled(True)
            if j_id in self.spinboxes:
                self.spinboxes[j_id].setEnabled(True)
            self.update_joint_display(j_id, angle, update_robot=False)

        # setChecked 신호를 막아 표시 과정에서 실제 토크 명령이 나가지 않게 합니다.
        for j_id, button in self.torque_btns.items():
            is_on = bool(torque_states.get(j_id, False))
            button.blockSignals(True)
            button.setChecked(is_on)
            button.setText("ON" if is_on else "OFF")
            button.setStyleSheet(
                ("background-color: #28a745;" if is_on else "background-color: #dc3545;")
                + " color: white; font-weight: bold; border-radius: 4px;"
            )
            button.blockSignals(False)
        self.update_torque_group_buttons()
        self.update_3d_robot(angles)
        self.tabs.setCurrentWidget(self.tab_frame)

        frame_name = active_frame.get("name", "프레임 사이") if active_frame else "프레임 사이"
        source_text = "실제 Present Position" if used_feedback else "타임라인 보간 목표"
        QMessageBox.information(
            self,
            "프레임 제작값 가져오기 완료",
            f"{capture_ms}ms / '{frame_name}' 구간의 자세를 가져왔습니다.\n"
            f"각도 기준: {source_text}\n"
            "토크 버튼은 상태만 표시했으며 실제 토크 명령은 보내지 않았습니다.\n"
            "필요한 관절을 수정한 뒤 '+ 프레임 추가'로 중간 비아포인트를 저장하세요.",
        )

    def queue_robot_scrub_target(self, angles_dict):
        """수동 타임라인 탐색 목표를 다음 200Hz tick에 그대로 전송합니다."""
        target = self.normalize_angles(angles_dict)
        if not target:
            return
        self.robot_scrub_target_angles = target
        if not self.robot_scrub_timer.isActive():
            self.robot_scrub_timer.start()

    def step_robot_scrub(self):
        if not self.robot_sync_enabled or not self.robot_scrub_target_angles:
            self.robot_scrub_timer.stop()
            return

        # 기존 45deg/s 클램프를 제거하고 타임라인이 요청한 각도를
        # 그대로 명령합니다. 물리적 최대 성능/토크 한계는 모터 자체가 관리합니다.
        next_angles = dict(self.robot_scrub_target_angles)
        target_ids = sorted(set(next_angles) & set(self.online_joints))

        # 직전 시퀀스의 도착시간이 남아 있으면 수동 탐색에도 그 시간이
        # 재사용되므로, 수동 Goal은 기존과 같은 즉시(step) 명령으로 되돌립니다.
        if (set(target_ids).issubset(self.time_based_profile_ids)
                and not self.configure_unlimited_motor_profiles(target_ids)):
            self.robot_scrub_timer.stop()
            QMessageBox.warning(self, "통신 에러", "수동 탐색용 모터 프로파일 초기화에 실패했습니다.")
            return

        if not self.write_goal_positions(
            next_angles,
            target_ids=target_ids,
        ):
            self.robot_scrub_timer.stop()
            QMessageBox.warning(self, "통신 에러", "타임라인 자세를 로봇에 전송하지 못했습니다.")
            return

        self.robot_scrub_command_angles = next_angles
        self.robot_scrub_timer.stop()

    def normalize_angles(self, angles_dict):
        normalized = {}
        if not angles_dict:
            return normalized
        for j_id, angle in angles_dict.items():
            try:
                normalized[int(j_id)] = float(angle)
            except (TypeError, ValueError):
                print(f"[⚠️ 각도 데이터 무시] 잘못된 joint 값: {j_id}={angle}")
        return normalized

    def angle_to_ui_value(self, angle_deg):
        return max(-180, min(180, int(round(float(angle_deg)))))

    def shortest_angle_delta(self, start_deg, end_deg):
        return ((float(end_deg) - float(start_deg) + 180.0) % 360.0) - 180.0

    def interpolate_angle_shortest(self, start_deg, end_deg, progress):
        return float(start_deg) + self.shortest_angle_delta(start_deg, end_deg) * float(progress)

    def frame_motion_progress(self, frame, t_ms):
        """GUI 실기 재생용 0→1 코사인 궤적 진행률을 계산합니다."""
        start_ms = int(frame.get("start_ms", 0))
        duration_ms = max(MIN_TIMELINE_FRAME_MS, int(frame.get("time_ms", 0)))
        elapsed_ms = max(0.0, min(float(t_ms - start_ms), float(duration_ms)))
        frame_name = str(frame.get("name", "")).lower()
        is_lift_frame = (
            bool(frame.get("lift_early_arrival", False))
            and any(
                keyword in frame_name
                for keyword in LIFT_FRAME_KEYWORDS
            )
        )
        arrival_ratio = (
            LIFT_TARGET_ARRIVAL_RATIO
            if is_lift_frame
            else 1.0
        )
        trajectory_duration_ms = max(
            1.0,
            float(duration_ms) * arrival_ratio,
        )
        linear_progress = min(1.0, elapsed_ms / trajectory_duration_ms)
        # 시작과 끝의 속도가 0이 되는 half-cosine 궤적입니다.
        # 발 들기 프레임은 앞 80%에서 progress=1이 된 뒤 남은 20% 동안
        # 정확한 최종 Goal을 유지합니다. 목표각을 넘겨 보내지는 않습니다.
        return 0.5 - 0.5 * math.cos(math.pi * linear_progress)

    def dxl_position_to_angle(self, dxl_position):
        angle_deg = (int(dxl_position) - DXL_CENTER_POSITION) * DXL_DEG_PER_STEP
        return max(DXL_MIN_DEG, min(DXL_MAX_DEG, angle_deg))

    def angle_to_dxl_position(self, angle_deg):
        angle_deg = float(angle_deg)
        while angle_deg > 180.0:
            angle_deg -= 360.0
        while angle_deg < -180.0:
            angle_deg += 360.0
        angle_deg = max(DXL_MIN_DEG, min(DXL_MAX_DEG, angle_deg))
        dxl_goal_position = int(round(DXL_CENTER_POSITION + angle_deg * DXL_STEPS_PER_DEG))
        return max(DXL_MIN_POSITION, min(DXL_MAX_POSITION, dxl_goal_position))

    def update_joint_display(self, j_id, angle_deg, update_robot=True):
        j_id = int(j_id)
        angle_deg = float(angle_deg)
        ui_angle = self.angle_to_ui_value(angle_deg)
        if j_id in self.sliders:
            self.sliders[j_id].blockSignals(True)
            self.sliders[j_id].setValue(ui_angle)
            self.sliders[j_id].blockSignals(False)
        if j_id in self.spinboxes:
            self.spinboxes[j_id].blockSignals(True)
            self.spinboxes[j_id].setValue(ui_angle)
            self.spinboxes[j_id].blockSignals(False)
        self.joints[j_id] = angle_deg
        if update_robot:
            self.update_3d_robot()

    def sequence_joint_ids(self):
        ids = set()
        for frame in self.motion_sequence:
            ids.update(self.normalize_angles(frame.get('angles', {})).keys())
        return ids

    def prepare_real_robot_sequence(self):
        if not self.port_opened:
            QMessageBox.warning(self, "통신 에러", "포트가 열려있지 않아 실제 로봇을 구동할 수 없습니다.")
            return False

        self.detect_online_joints()
        if not self.online_joints:
            QMessageBox.warning(self, "통신 에러", "온라인 모터가 없어 실제 로봇을 구동할 수 없습니다.")
            return False

        sequence_ids = self.sequence_joint_ids()
        if not sequence_ids:
            QMessageBox.warning(self, "경고", "시퀀스에 관절 각도 데이터가 없습니다.")
            return False

        missing_ids = sorted(sequence_ids - set(self.online_joints))
        if missing_ids:
            QMessageBox.warning(self, "통신 에러", f"시퀀스에 포함된 모터 중 오프라인 ID가 있습니다.\n오프라인: {missing_ids}\n모터 재탐색 후 다시 실행하세요.")
            return False

        torque_off_ids = []
        torque_read_failed_ids = []
        torque_states = {}
        for j_id in sorted(sequence_ids):
            torque_state = self.read_torque_enabled(j_id)
            torque_states[j_id] = torque_state
            if torque_state is False:
                torque_off_ids.append(j_id)
            elif torque_state is None:
                torque_read_failed_ids.append(j_id)

        if torque_read_failed_ids:
            QMessageBox.warning(
                self,
                "통신 에러",
                f"토크 상태를 확인하지 못한 모터가 있어 실제 구동을 중단합니다.\nID: {torque_read_failed_ids}",
            )
            return False

        not_ready_ids = sorted(sequence_ids - self.time_based_profile_ids)
        if not_ready_ids:
            torque_on_not_ready = [
                j_id for j_id in not_ready_ids if torque_states.get(j_id) is True
            ]
            if torque_on_not_ready:
                QMessageBox.warning(
                    self,
                    "시간 기반 프로파일 준비 필요",
                    "프레임 시간대로 도착하도록 설정하려면 Drive Mode EEPROM을 "
                    "변경해야 하므로 먼저 전체 토크를 OFF해야 합니다.\n"
                    f"설정되지 않은 토크 ON ID: {torque_on_not_ready}",
                )
                return False
            if not self.configure_time_based_drive_mode(not_ready_ids):
                QMessageBox.warning(
                    self,
                    "시간 기반 프로파일 설정 실패",
                    "MX 펌웨어 V42 이상 및 토크 OFF 상태가 필요합니다.\n"
                    f"설정 실패 ID: {not_ready_ids}",
                )
                return False

        # 이전 재생에서 남은 프레임 도착시간을 지우고, 첫 프레임이 시작될 때
        # 그 프레임의 실제 재생시간을 새로 기록합니다.
        if not self.configure_unlimited_motor_profiles(sequence_ids):
            QMessageBox.warning(
                self,
                "통신 에러",
                "시간 기반 모터 프로파일을 초기화하지 못해 실제 구동을 중단합니다.",
            )
            return False

        if torque_off_ids:
            print(f"   [🔒 실제 구동 준비] 토크 OFF 감지 ID {torque_off_ids} -> 전체 토크 ON 시도")
            self.set_all_torque_on()

        still_off_ids = []
        verify_failed_ids = []
        for j_id in sorted(sequence_ids):
            torque_state = self.read_torque_enabled(j_id)
            if torque_state is False:
                still_off_ids.append(j_id)
            elif torque_state is None:
                verify_failed_ids.append(j_id)

        if still_off_ids or verify_failed_ids:
            QMessageBox.warning(
                self,
                "통신 에러",
                "토크 ON 확인에 실패한 모터가 있어 실제 구동을 중단합니다.\n"
                f"OFF: {still_off_ids}\n상태 확인 실패: {verify_failed_ids}",
            )
            return False

        return True

    def angles_to_urdf_radians(self, angles_dict):
        """DYNAMIXEL ID 기준 각도(deg)를 URDF joint 기준 각도(rad)로 변환합니다."""
        angles = self.normalize_angles(angles_dict)
        return {
            urdf_name: math.radians(angles[j_id])
            for j_id, urdf_name in self.urdf_joint_map.items()
            if j_id in angles
        }

    def calculate_fk_tracking_error(self):
        """명령/실측 각도의 FK와 관절 추종 오차를 계산합니다."""
        import numpy as np

        # 아직 Goal을 보내지 않은 축은 실측값을 목표 자세의 기준으로
        # 사용해, 부분 관절 명령 시 무관한 축이 FK 오차에 포함되지 않게 합니다.
        target_angles = dict(self.feedback_angles)
        for j_id in self.commanded_joint_ids:
            if j_id in self.commanded_angles:
                target_angles[j_id] = self.commanded_angles[j_id]

        joint_errors = {
            j_id: self.shortest_angle_delta(
                self.feedback_angles[j_id],
                self.commanded_angles[j_id],
            )
            for j_id in self.commanded_joint_ids
            if j_id in self.feedback_angles and j_id in self.commanded_angles
        }

        result = {
            "joint_errors_deg": joint_errors,
            "joint_samples": {
                j_id: {
                    "goal_deg": self.commanded_angles[j_id],
                    "actual_deg": self.feedback_angles[j_id],
                    "error_deg": error,
                }
                for j_id, error in joint_errors.items()
            },
            "joint_rms_deg": 0.0,
            "joint_max_deg": 0.0,
            "joint_max_id": None,
            "end_effectors": {},
        }
        if joint_errors:
            max_joint_id = max(joint_errors, key=lambda j_id: abs(joint_errors[j_id]))
            result["joint_rms_deg"] = math.sqrt(
                sum(error * error for error in joint_errors.values()) / len(joint_errors)
            )
            result["joint_max_deg"] = abs(joint_errors[max_joint_id])
            result["joint_max_id"] = max_joint_id

        if not self.urdf_loaded or self.robot_model is None:
            return result

        target_poses = self.robot_model._compute_link_poses(
            self.angles_to_urdf_radians(target_angles)
        )
        actual_poses = self.robot_model._compute_link_poses(
            self.angles_to_urdf_radians(self.feedback_angles)
        )
        for link_name in self.robot_model.end_effector_links:
            target_pose = target_poses.get(link_name)
            actual_pose = actual_poses.get(link_name)
            if target_pose is None or actual_pose is None:
                continue

            position_error_mm = float(
                np.linalg.norm(target_pose[:3, 3] - actual_pose[:3, 3]) * 1000.0
            )
            relative_rotation = target_pose[:3, :3].T @ actual_pose[:3, :3]
            cos_angle = float(np.clip((np.trace(relative_rotation) - 1.0) * 0.5, -1.0, 1.0))
            orientation_error_deg = math.degrees(math.acos(cos_angle))
            result["end_effectors"][link_name] = {
                "position_mm": position_error_mm,
                "orientation_deg": orientation_error_deg,
            }

        return result

    def print_fk_tracking_error(self, error_data, read_elapsed_ms):
        """고속 피드백 계산 결과를 제어를 방해하지 않는 10Hz 요약으로 출력합니다."""
        now = time.perf_counter()
        if (self.fk_feedback_sample_count > 1
                and now - self.last_fk_error_log_time < FK_ERROR_LOG_INTERVAL_SEC):
            return
        self.last_fk_error_log_time = now

        max_joint_id = error_data["joint_max_id"]
        max_joint_text = "-" if max_joint_id is None else str(max_joint_id)
        fk_parts = []
        for link_name, error in error_data["end_effectors"].items():
            fk_parts.append(
                f"{link_name}={error['position_mm']:.2f}mm/{error['orientation_deg']:.2f}deg"
            )
        fk_text = " | ".join(fk_parts) if fk_parts else "FK unavailable"
        joint_text = " ".join(
            f"{j_id}:{sample['goal_deg']:.2f}/{sample['actual_deg']:.2f}/{sample['error_deg']:+.2f}"
            for j_id, sample in sorted(error_data["joint_samples"].items())
        )
        print(f"[실기 관절 goal/actual/error deg] {joint_text}")
        print(
            f"[실기 Present Position FK 오차 #{self.fk_feedback_sample_count:06d}] "
            f"joint RMS={error_data['joint_rms_deg']:.3f}deg, "
            f"MAX={error_data['joint_max_deg']:.3f}deg(ID {max_joint_text}) | "
            f"{fk_text} | SyncRead={read_elapsed_ms:.2f}ms"
        )

    def read_feedback_and_report_fk(self):
        """전 관절 Present Position을 SyncRead하고 현재 타임라인 FK 오차를 계산합니다."""
        read_ids = sorted(set(self.online_joints))
        if not read_ids:
            return False

        started_at = time.perf_counter()
        self.groupSyncRead.clearParam()
        for j_id in read_ids:
            if not self.groupSyncRead.addParam(j_id):
                print(f"[❌ FK 피드백 준비 실패] ID {j_id}")
                return False

        dxl_comm_result = self.groupSyncRead.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.fk_feedback_read_failures += 1
            now = time.perf_counter()
            if now - self.last_fk_read_error_log_time >= 1.0:
                self.last_fk_read_error_log_time = now
                print(
                    f"[❌ FK 피드백 실패 x{self.fk_feedback_read_failures}] "
                    f"{self.packetHandler.getTxRxResult(dxl_comm_result)}"
                )
            return False

        measured_angles = {}
        for j_id in read_ids:
            if not self.groupSyncRead.isAvailable(
                j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            ):
                continue
            dxl_present_position = self.groupSyncRead.getData(
                j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
            )
            measured_angles[j_id] = self.dxl_position_to_angle(dxl_present_position)

        missing_ids = sorted(set(read_ids) - set(measured_angles))
        if missing_ids:
            self.fk_feedback_read_failures += 1
            now = time.perf_counter()
            if now - self.last_fk_read_error_log_time >= 1.0:
                self.last_fk_read_error_log_time = now
                print(
                    f"[❌ 실기 Present Position 누락 x{self.fk_feedback_read_failures}] "
                    f"ID {missing_ids} - FK 오차 계산 생략"
                )
            return False

        self.fk_feedback_read_failures = 0
        self.feedback_angles.update(measured_angles)
        self.fk_feedback_sample_count += 1
        error_data = self.calculate_fk_tracking_error()
        read_elapsed_ms = (time.perf_counter() - started_at) * 1000.0
        self.print_fk_tracking_error(error_data, read_elapsed_ms)
        return True

    def write_goal_positions(self, angles_dict, target_ids=None, read_feedback=True):
        if not self.port_opened:
            return False

        angles = self.normalize_angles(angles_dict)
        if not angles:
            return False

        ids = list(target_ids) if target_ids is not None else list(self.online_joints)
        self.groupSyncWrite.clearParam()
        added_count = 0
        sent_angles = {}
        for j_id in ids:
            j_id = int(j_id)
            if j_id not in angles:
                continue
            dxl_goal_position = self.angle_to_dxl_position(angles[j_id])
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))
            ]
            if self.groupSyncWrite.addParam(j_id, param_goal_position):
                added_count += 1
                # FK 목표는 모터에 실제 전송된 quantize/clamp 각도를 사용합니다.
                sent_angles[j_id] = self.dxl_position_to_angle(dxl_goal_position)
            else:
                print(f"[❌ SyncWrite 준비 실패] ID {j_id} 파라미터 추가 실패")

        if added_count == 0:
            return False

        dxl_comm_result = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[❌ SyncWrite 에러] 목표 위치 전송 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False

        self.commanded_angles.update(sent_angles)
        self.commanded_joint_ids.update(sent_angles.keys())
        # 시간 기반 프로파일은 프레임 목표를 먼저 한 번 전송한 뒤, 현재
        # 타임라인 목표를 별도로 기록하고 피드백을 읽습니다.
        if read_feedback:
            # 피드백 읽기/FK 실패는 이미 성공한 Goal Position 전송 결과를
            # 실패로 바꾸지 않습니다.
            self.read_feedback_and_report_fk()
        return True

    def set_tracking_target_angles(self, angles_dict):
        """FK/관절 오차의 목표를 현재 타임라인상의 자세로 갱신합니다."""
        for j_id, angle in self.normalize_angles(angles_dict).items():
            dxl_position = self.angle_to_dxl_position(angle)
            self.commanded_angles[j_id] = self.dxl_position_to_angle(dxl_position)
            self.commanded_joint_ids.add(j_id)

    def gate_timeline_on_frame_arrival(self, requested_t_ms, now):
        """프레임 종료시각에 최종 Goal을 확정하고 기다리지 않고 진행합니다."""
        frame = self.timed_gate_frame
        if frame is None:
            return requested_t_ms
        frame_end_ms = int(frame.get("start_ms", 0)) + max(
            CONTROL_INTERVAL_MS,
            int(frame.get("time_ms", CONTROL_INTERVAL_MS)),
        )
        if requested_t_ms < frame_end_ms:
            return requested_t_ms

        frame_angles = self.normalize_angles(frame.get("angles", {}))
        target_ids = sorted(set(frame_angles) & set(self.online_joints))
        if not target_ids or not self.write_goal_positions(
            frame_angles,
            target_ids=target_ids,
            read_feedback=False,
        ):
            self.is_playing = False
            self.is_paused = True
            self.anim_timer.stop()
            self.update_playback_buttons()
            QMessageBox.critical(
                self,
                "최종 목표 전송 실패",
                f"'{frame.get('name', 'Frame')}'의 최종 Goal Position을 "
                "전송하지 못해 재생을 중단합니다.",
            )
            return None

        print(
            f"[🎯 지정시각 최종각 전송] '{frame.get('name', 'Frame')}' "
            f"도착명령={frame_end_ms}ms"
        )
        self.timed_gate_frame = None
        self.timed_gate_wait_started = 0.0
        self.timed_gate_last_check = 0.0
        # 타이머 지터로 경계를 조금 넘었더라도 이번 tick은 정확한 경계
        # 자세만 처리합니다. 다음 tick부터 다음 프레임의 코사인 궤적을 보냅니다.
        return frame_end_ms

    def command_time_profile_frame(self, frame, t_ms):
        """프레임 종료각을 남은 실제 시간과 함께 모터에 한 번만 전송합니다."""
        frame_key = frame.get("frame_id") or id(frame)
        token = (
            self.playback_repeat_current,
            frame_key,
            int(frame.get("start_ms", 0)),
            int(frame.get("time_ms", CONTROL_INTERVAL_MS)),
        )
        if token == self.active_timed_frame_token:
            return True, False

        frame_angles = self.normalize_angles(frame.get("angles", {}))
        target_ids = sorted(set(frame_angles) & set(self.online_joints))
        if not target_ids:
            self.active_timed_frame_token = token
            return True, False
        if not set(target_ids).issubset(self.time_based_profile_ids):
            print(f"[❌ 시간 프로파일 미준비] ID {sorted(set(target_ids) - self.time_based_profile_ids)}")
            return False, False

        frame_end_ms = int(frame.get("start_ms", 0)) + max(
            CONTROL_INTERVAL_MS,
            int(frame.get("time_ms", CONTROL_INTERVAL_MS)),
        )
        remaining_timeline_ms = max(1, frame_end_ms - int(t_ms))
        real_duration_ms = max(1, int(math.ceil(remaining_timeline_ms / self.playback_speed)))
        frame_name = str(frame.get("name", ""))
        is_landing_frame = LANDING_FRAME_TAG in frame_name.lower()
        accel_ms = 0
        if is_landing_frame:
            accel_ms = min(
                int(real_duration_ms * LANDING_ACCEL_RATIO),
                LANDING_ACCEL_MAX_MS,
            )
        if not self.set_time_profile_duration(
            target_ids,
            real_duration_ms,
            accel_ms=accel_ms,
        ):
            return False, False
        if not self.write_goal_positions(
            frame_angles,
            target_ids=target_ids,
            read_feedback=False,
        ):
            return False, False

        self.active_timed_frame_token = token
        self.timed_gate_frame = frame
        self.timed_gate_wait_started = 0.0
        self.timed_gate_last_check = 0.0
        required_speeds = {
            j_id: abs(self.shortest_angle_delta(
                self.feedback_angles.get(j_id, frame_angles[j_id]),
                frame_angles[j_id],
            )) * 1000.0 / real_duration_ms
            for j_id in target_ids
        }
        max_speed_id = max(required_speeds, key=required_speeds.get)
        print(
            f"[⏱️ 시간지정 프레임] '{frame.get('name', 'Frame')}' "
            f"timeline_remaining={remaining_timeline_ms}ms, "
            f"motor_arrival={real_duration_ms}ms, accel={accel_ms}ms, "
            f"landing={'yes' if is_landing_frame else 'no'}, "
            f"speed={self.playback_speed:.2f}x, "
            f"required_max={required_speeds[max_speed_id]:.1f}deg/s(ID {max_speed_id})"
        )
        return True, True

    def hold_timed_motion_at_feedback(self):
        """일시정지/사용자 중지 시 예약된 최종 목표로 계속 가지 않게 즉시 고정합니다."""
        if not self.execute_on_real_robot or self.active_timed_frame_token is None:
            return
        target_ids = sorted(self.sequence_joint_ids() & set(self.online_joints))
        if not target_ids or not set(target_ids).issubset(self.time_based_profile_ids):
            return

        self.read_feedback_and_report_fk()
        hold_angles = {
            j_id: self.feedback_angles[j_id]
            for j_id in target_ids
            if j_id in self.feedback_angles
        }
        if not hold_angles:
            return
        if self.configure_unlimited_motor_profiles(hold_angles.keys()):
            self.write_goal_positions(
                hold_angles,
                target_ids=hold_angles.keys(),
                read_feedback=False,
            )

    def read_present_angle(self, j_id):
        if not hasattr(self, 'port_opened') or not self.port_opened:
            return None

        # TxOnly 쓰기의 상태 패킷이 늦게 도착하거나 읽기 응답이 순간적으로
        # 잘리는 경우가 있으므로 버퍼를 비운 뒤 최대 3회 다시 읽습니다.
        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.005)
                self.portHandler.clearPort()

            dxl_present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
                self.portHandler, j_id, ADDR_PRESENT_POSITION
            )
            if dxl_comm_result == COMM_SUCCESS:
                break
        else:
            print(f"[❌ Read 에러] ID {j_id} 현재 각도 읽기 3회 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None

        if dxl_error != 0:
            # Hardware Alert가 있어도 Present Position 데이터 자체는 정상
            # 수신되므로 시작 자세 계산에는 실제 위치 값을 사용합니다.
            print(
                f"[⚠️ 모터 하드웨어 경고] ID {j_id}: "
                f"{self.packetHandler.getRxPacketError(dxl_error)} "
                "(현재 위치 값은 정상 수신)"
            )

        if j_id not in self.online_joints:
            self.online_joints.append(j_id)
            self.online_joints.sort()

        return self.dxl_position_to_angle(dxl_present_position)

    def read_torque_enabled(self, j_id):
        if not hasattr(self, 'port_opened') or not self.port_opened:
            return None

        for attempt in range(3):
            if attempt > 0:
                time.sleep(0.005)
                self.portHandler.clearPort()
            torque_value, dxl_comm_result, dxl_error = self.packetHandler.read1ByteTxRx(
                self.portHandler, j_id, ADDR_TORQUE_ENABLE
            )
            if dxl_comm_result == COMM_SUCCESS:
                break

        if dxl_comm_result != COMM_SUCCESS:
            print(f"[❌ Read 에러] ID {j_id} 토크 상태 3회 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return None
        if dxl_error != 0:
            # Protocol 2.0의 Alert/Hardware Error 비트는 읽기 통신 실패가
            # 아닙니다. 반환된 Torque Enable 값은 유효하므로 상태 판정에는
            # 사용하되, 실제 과부하/과열/전압 문제는 별도 경고로 남깁니다.
            print(
                f"[⚠️ 모터 하드웨어 경고] ID {j_id}: "
                f"{self.packetHandler.getRxPacketError(dxl_error)} "
                f"(토크 상태 값은 정상 수신: {'ON' if torque_value == 1 else 'OFF'})"
            )

        return torque_value == 1

    # 🚀 [원인 분석 및 해결] 23개 모터 응답 충돌(Incorrect status packet)을 막기 위해 TxOnly 사용!
    def sync_torque_group(self, group_name, joint_ids, is_on):
        print(f"👉 [그룹 토크] {group_name} 모터 전체 {'ON' if is_on else 'OFF'} 요청")

        for j_id in joint_ids:
            btn = self.torque_btns.get(j_id)
            if btn is not None and btn.isChecked() != is_on:
                # 기존 개별 토크 안전 절차(현재 위치 동기화 포함)를 그대로 사용합니다.
                btn.setChecked(is_on)

        self.update_torque_group_buttons()

    def update_torque_group_buttons(self):
        for group_name, joint_ids in self.torque_groups.items():
            btn_group = self.torque_group_btns.get(group_name)
            if btn_group is None:
                continue

            all_on = all(
                j_id in self.torque_btns and self.torque_btns[j_id].isChecked()
                for j_id in joint_ids
            )
            btn_group.blockSignals(True)
            btn_group.setChecked(all_on)
            btn_group.setText(f"{group_name} 전체 {'ON' if all_on else 'OFF'}")
            btn_group.setStyleSheet(
                ("background-color: #28a745;" if all_on else "background-color: #dc3545;")
                + " color: white; font-weight: bold; font-size: 11pt;"
            )
            btn_group.blockSignals(False)

    def sync_torque(self, j_id, is_on):
        print(f"👉 [GUI 작동 감지] {j_id}번 모터 토크 {'ON' if is_on else 'OFF'} 스위치 조작됨!")
        
        btn = self.torque_btns[j_id]

        if hasattr(self, 'port_opened') and self.port_opened:
            if is_on:
                # 직전 OFF/Goal TxOnly 명령에 대한 늦은 상태 패킷을 제거합니다.
                time.sleep(0.005)
                self.portHandler.clearPort()

                # 토크 ON 전 실제 Present Position을 먼저 읽어 GUI와 Goal Position을 맞춥니다.
                # 시작 시 online_joints 감지가 비어 있어도 여기서 직접 읽기를 시도합니다.
                angle_deg = self.read_present_angle(j_id)
                if angle_deg is None:
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(j_id in self.online_joints)
                    self.spinboxes[j_id].setEnabled(j_id in self.online_joints)
                    QMessageBox.warning(self, "통신 에러", f"ID {j_id} 모터의 현재 각도를 읽지 못해 토크 ON을 중단했습니다.")
                    return

                self.update_joint_display(j_id, angle_deg)
                self.feedback_angles[j_id] = angle_deg
                self.commanded_angles[j_id] = angle_deg
                self.commanded_joint_ids.add(j_id)

                # 목표 주소(Goal Position)에 튐 방지용 현재 위치를 선등록
                dxl_goal_position = self.angle_to_dxl_position(angle_deg)
                dxl_comm_result = self.packetHandler.write4ByteTxOnly(self.portHandler, j_id, ADDR_GOAL_POSITION, dxl_goal_position)
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"[❌ TX 에러] ID {j_id} Goal 선등록 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(j_id in self.online_joints)
                    self.spinboxes[j_id].setEnabled(j_id in self.online_joints)
                    QMessageBox.warning(self, "통신 에러", f"ID {j_id} 모터의 Goal Position 선등록에 실패해 토크 ON을 중단했습니다.")
                    return

                # Goal Position 쓰기 응답이 다음 Torque 쓰기와 섞이지 않게 비웁니다.
                time.sleep(0.005)
                self.portHandler.clearPort()

                if not self.configure_unlimited_motor_profiles([j_id]):
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(j_id in self.online_joints)
                    self.spinboxes[j_id].setEnabled(j_id in self.online_joints)
                    QMessageBox.warning(
                        self,
                        "통신 에러",
                        f"ID {j_id} 모터의 속도/가속도 프로파일 설정에 실패해 "
                        "토크 ON을 중단했습니다.",
                    )
                    return

                print(f"   [🔄 동기화 완료] 개별 토크 ON 전 실제 각도 {angle_deg}도 획득 및 Goal 선매핑")

            torque_val = 1 if is_on else 0
            dxl_comm_result = self.packetHandler.write1ByteTxOnly(self.portHandler, j_id, ADDR_TORQUE_ENABLE, torque_val)

            # 모터의 Status Return Level 설정에 따라 TxOnly에도 응답이 올 수
            # 있으므로 다음 읽기 전에 남지 않도록 수신 버퍼를 정리합니다.
            time.sleep(0.005)
            self.portHandler.clearPort()
            
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[❌ TX 에러] ID {j_id} 토크 제어 전송 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                if is_on:
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.blockSignals(False)
                    is_on = False
            else:
                print(f"[✅ 통신 성공] ID {j_id} 토크 {'ON' if is_on else 'OFF'} 명령 쏨 (응답 무시)")

        btn.setText("ON" if is_on else "OFF")
        btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px;" if is_on else "background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
        self.sliders[j_id].setEnabled(j_id in self.online_joints)
        self.spinboxes[j_id].setEnabled(j_id in self.online_joints)
        self.update_torque_group_buttons()

    # 수동 Goal Position도 공통 SyncWrite -> 실기 SyncRead -> FK 경로를 사용합니다.
    def sync_values(self, joint_id, value, source, transmit_motor=True):
        if transmit_motor and self.frame_apply_timer.isActive():
            self.frame_apply_timer.stop()
            self.frame_apply_ids = []
            self.frame_apply_frame = None
            self.frame_apply_on_complete = None

        # 사용자가 숫자/슬라이더를 직접 조작한 값은 프레임 편집 목표이므로
        # 실기 라이브 피드백이 다시 덮어쓰지 않게 유지합니다.
        self.editing_loaded_pose = True
        self.joints[joint_id] = value
        if source == 'slider':
            self.spinboxes[joint_id].blockSignals(True)
            self.spinboxes[joint_id].setValue(value)
            self.spinboxes[joint_id].blockSignals(False)
        elif source == 'spinbox':
            self.sliders[joint_id].blockSignals(True)
            self.sliders[joint_id].setValue(value)
            self.sliders[joint_id].blockSignals(False)
        self.update_3d_robot()
        
        if (transmit_motor and self.torque_btns[joint_id].isChecked()
                and hasattr(self, 'port_opened') and self.port_opened):
            # 직전 시퀀스의 time-based 도착시간이 남아 있으면 수동 입력에도
            # 적용되므로, Enter 수동 명령은 step 프로파일로 초기화합니다.
            if (joint_id in self.time_based_profile_ids
                    and not self.configure_unlimited_motor_profiles([joint_id])):
                print(f"[❌ TX 에러] ID {joint_id} 수동 입력 프로파일 초기화 실패")
                return
            if not self.write_goal_positions({joint_id: value}, target_ids=[joint_id]):
                print(f"[❌ TX 에러] ID {joint_id} 각도 전송 실패")
            else:
                print(f"[✅ 통신 성공] ID {joint_id} -> {value}도 명령 후 실기 Present Position 읽기")

    def read_angles_from_robot(self):
        print("👉 [GUI 작동 감지] 실제 로봇 관절값 읽어오기 버튼 눌림!")
        if not self.port_opened: return QMessageBox.warning(self, "에러", "포트가 열려있지 않습니다.")
        
        # 🚀 [완벽 패치] 로봇 전원이 그새 켜졌을 수 있으므로 온라인 모터 목록을 새로 갱신합니다.
        self.detect_online_joints()
        if not self.online_joints:
             QMessageBox.warning(self, "에러", "통신 가능한 다이나믹셀 모터가 존재하지 않습니다.\n배터리 및 배선 상태를 재점검하세요.")
             return

        if QMessageBox.question(self, 'Teach Mode', f"감지된 {len(self.online_joints)}개 관절의 실제 각도를 가져오시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.No: return
        
        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)
            
        # ⚠️ 값을 '읽어올' 때는 대답을 기다려야 하므로 TxRx를 그대로 둡니다. (이건 Return Level 1에서도 정상 작동함)
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        
        if dxl_comm_result != COMM_SUCCESS:
            print(f"[❌ Read 에러] 로봇 값 읽어오기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            QMessageBox.warning(self, "통신 에러", "물리 로봇에서 데이터를 읽어오는 데 실패했습니다.\n터미널 로그를 확인하세요.")
            return
            
        success_ids = []
        self.editing_loaded_pose = False
        for j_id in self.online_joints:
            if self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                dxl_present_position = self.groupSyncRead.getData(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                angle_deg = self.dxl_position_to_angle(dxl_present_position)
                self.update_joint_display(j_id, angle_deg, update_robot=False)
                success_ids.append(j_id)
        
        print(f"[✅ Read 성공] 다음 ID의 값을 성공적으로 읽어왔습니다: {success_ids}")        
        self.update_3d_robot()
        QMessageBox.information(self, "완료", f"실제 관절 {len(success_ids)}개의 현재 각도를 성공적으로 매핑했습니다!")

    def apply_to_real_robot(self, angles_dict, force=False):
        if not self.execute_on_real_robot or not self.port_opened: return
        # 호출자인 PreciseTimer가 5ms(200Hz) 주기를 관리합니다.
        # 기존 30ms 소프트웨어 쓰기 제한은 제거했습니다.
        self.write_goal_positions(angles_dict, read_feedback=False)

    # 🚀 [제로-저크 안전 결합 해결] 전체 토크를 켤 때, 먼저 23축 전체 물리 각도를 SyncRead로 긁어와 목표 각도(Goal Position)를 강제 일치시킵니다.
    def set_all_torque_on(self):
        print("👉 [GUI 작동 감지] 전체 토크 ON 버튼 눌림!")
        if not self.port_opened: return
        
        # 🚀 [완벽 패치] 프로그램 기동 시 로봇 전원이 꺼져 있었더라도, 토크 ON 시점에 무조건 자동 재수색하여 활성화합니다!
        if not self.online_joints:
            print("   [🔍 온라인 모터 재수색] 기동 시 감지된 관절이 없어 재탐색을 수행합니다...")
            self.detect_online_joints()
            if not self.online_joints:
                QMessageBox.warning(self, "통신 에러", "현재 연결된 다이나믹셀 모터를 스캔해내지 못했습니다.\n배터리/SMPS 전원 스위치가 단단히 켜져 있는지 확인하세요.")
                return

        # 1. 온라인이 확정된 축의 현재 실제 각도를 초고속으로 일괄 스캔하여 GUI 및 변수에 동기화
        self.groupSyncRead.clearParam()
        for j_id in self.online_joints:
            self.groupSyncRead.addParam(j_id)
            
        dxl_comm_result = self.groupSyncRead.txRxPacket()
        read_success_ids = set()
        if dxl_comm_result == COMM_SUCCESS:
            print("   [✅ Read 성공] 일제 잠금 전 전신 관절값 스캔 성공!")
            for j_id in self.online_joints:
                if self.groupSyncRead.isAvailable(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    dxl_present_position = self.groupSyncRead.getData(j_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
                    angle_deg = self.dxl_position_to_angle(dxl_present_position)
                    self.update_joint_display(j_id, angle_deg, update_robot=False)
                    read_success_ids.add(j_id)
        else:
            print(f"   [⚠️ 경고] 토크 ON 전 각도 읽기 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")

        missing_ids = [j_id for j_id in self.online_joints if j_id not in read_success_ids]
        if missing_ids:
            print(f"   [🔁 개별 Read 재시도] SyncRead 누락 ID: {missing_ids}")
            for j_id in missing_ids:
                angle_deg = self.read_present_angle(j_id)
                if angle_deg is not None:
                    self.update_joint_display(j_id, angle_deg, update_robot=False)
                    read_success_ids.add(j_id)

        if not read_success_ids:
            QMessageBox.warning(self, "통신 에러", "토크 ON 전 현재 각도를 읽어오지 못해 전체 토크 ON을 중단했습니다.")
            return

        actual_start_angles = {j_id: self.joints[j_id] for j_id in read_success_ids}
        self.feedback_angles.update(actual_start_angles)
        self.commanded_angles.update(actual_start_angles)
        self.commanded_joint_ids.update(read_success_ids)
        self.update_3d_robot()

        # 2. 새로 연결된 모터도 토크를 켜기 전에 시간 기반 Drive Mode를 준비합니다.
        not_ready_ids = sorted(set(read_success_ids) - self.time_based_profile_ids)
        if not_ready_ids:
            torque_on_ids = [
                j_id for j_id in not_ready_ids
                if self.read_torque_enabled(j_id) is True
            ]
            if torque_on_ids or not self.configure_time_based_drive_mode(not_ready_ids):
                QMessageBox.warning(
                    self,
                    "시간 기반 프로파일 설정 실패",
                    "Drive Mode 설정은 토크 OFF 상태에서만 가능합니다.\n"
                    f"확인할 ID: {torque_on_ids or not_ready_ids}",
                )
                return

        # 3. 잠금 전 Profile 값을 0으로 초기화합니다. 시간 기반 모드에서 0은
        # 선등록 Goal을 현재각에 즉시 맞추는 step 설정이며 실제 이동 프레임은
        # 재생 시 해당 time_ms로 다시 설정됩니다.
        if not self.configure_unlimited_motor_profiles(read_success_ids):
            QMessageBox.warning(
                self,
                "통신 에러",
                "모터 속도/가속도 프로파일 설정에 실패해 전체 토크 ON을 중단했습니다.",
            )
            return

        # 4. 동기화된 각도 데이터를 모든 Goal Position 레지스터에 선등록하여 잠금 시 snap(발작) 발생을 원천 차단
        self.groupSyncWrite.clearParam()
        for j_id in self.online_joints:
            if j_id not in read_success_ids:
                continue
            angle_deg = self.joints[j_id]
            dxl_goal_position = self.angle_to_dxl_position(angle_deg)
            param_goal_position = [
                DXL_LOBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_LOWORD(dxl_goal_position)),
                DXL_LOBYTE(DXL_HIWORD(dxl_goal_position)),
                DXL_HIBYTE(DXL_HIWORD(dxl_goal_position))
            ]
            self.groupSyncWrite.addParam(j_id, param_goal_position)
        dxl_goal_result = self.groupSyncWrite.txPacket()
        if dxl_goal_result != COMM_SUCCESS:
            QMessageBox.warning(self, "통신 에러", f"Goal Position 선등록 실패로 전체 토크 ON을 중단했습니다.\n{self.packetHandler.getTxRxResult(dxl_goal_result)}")
            return

        # 5. 그 다음 전 관절 잠금 (Torque ON)
        self.groupSyncWriteTorque.clearParam()
        for j_id, btn in self.torque_btns.items():
            btn.blockSignals(True)
            if j_id in read_success_ids:
                btn.setEnabled(True)
                btn.setChecked(True)
                btn.setText("ON")
                btn.setStyleSheet("background-color: #28a745; color: white; font-weight: bold; border-radius: 4px;")
                self.sliders[j_id].setEnabled(True)
                self.spinboxes[j_id].setEnabled(True)
                self.groupSyncWriteTorque.addParam(j_id, [1])
            else:
                # 통신 불가능 관절은 UI 비활성화 유지
                btn.setEnabled(False)
                btn.setChecked(False)
                btn.setText("OFF")
                btn.setStyleSheet("background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px;")
                self.sliders[j_id].setEnabled(False)
                self.spinboxes[j_id].setEnabled(False)
            btn.blockSignals(False)
        
        if read_success_ids:
            dxl_comm_result = self.groupSyncWriteTorque.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"   [❌ 에러] 전체 토크 ON 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
                for j_id in read_success_ids:
                    btn = self.torque_btns[j_id]
                    btn.blockSignals(True)
                    btn.setChecked(False)
                    btn.setText("OFF")
                    btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;")
                    self.sliders[j_id].setEnabled(j_id in self.online_joints)
                    self.spinboxes[j_id].setEnabled(j_id in self.online_joints)
                    btn.blockSignals(False)
                QMessageBox.warning(self, "통신 에러", f"전체 토크 ON 명령 전송에 실패했습니다.\n{self.packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                print(f"   [✅ 발송 완료] 현재 각도 읽기에 성공한 {len(read_success_ids)}개 관절에 잠금 명령 쐈습니다!")

        self.update_torque_group_buttons()

    def set_all_torque_off(self):
        print("👉 [GUI 작동 감지] 전체 토크 OFF 버튼 눌림!")
        if not self.port_opened: return
        self.groupSyncWriteTorque.clearParam()
        for j_id, btn in self.torque_btns.items():
            btn.blockSignals(True)
            btn.setEnabled(j_id in self.online_joints)
            btn.setChecked(False)
            btn.setText("OFF")
            btn.setStyleSheet("background-color: #dc3545; color: white; font-weight: bold; border-radius: 4px;" if j_id in self.online_joints else "background-color: #6c757d; color: white; font-weight: bold; border-radius: 4px;")
            self.sliders[j_id].setEnabled(j_id in self.online_joints)
            self.spinboxes[j_id].setEnabled(j_id in self.online_joints)
            btn.blockSignals(False)
            
            if j_id in self.online_joints:
                self.groupSyncWriteTorque.addParam(j_id, [0])
            
        if self.online_joints:
            dxl_comm_result = self.groupSyncWriteTorque.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print(f"[❌ 에러] 전체 토크 OFF 실패: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            else:
                print(f"[✅ 발송 완료] 감지된 {len(self.online_joints)}개 모터에 티칭(늘어짐) 명령 쐈습니다!")
                if not self.configure_time_based_drive_mode(self.online_joints):
                    print(
                        "[⚠️ 시간 기반 프로파일 준비 실패] "
                        "실기 시퀀스 재생 전에 모터 펌웨어와 Drive Mode를 확인하세요."
                    )

        self.update_torque_group_buttons()

    def sync_drag_selection(self):
        if not self.is_select_mode: return
        selected_items = self.frame_list_ui1.selectedItems()
        for i in range(self.frame_list_ui1.count()):
            widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(i))
            if widget:
                widget.checkbox.blockSignals(True)
                widget.checkbox.setChecked(self.frame_list_ui1.item(i) in selected_items)
                widget.checkbox.blockSignals(False)

    def update_frame_library_selection_style(self, current, previous):
        """실제로 실행될 프레임 행을 색으로 명확하게 표시합니다."""
        if previous is not None:
            previous_widget = self.frame_list_ui1.itemWidget(previous)
            if previous_widget is not None:
                previous_widget.set_selected_style(False)
        if current is not None:
            current_widget = self.frame_list_ui1.itemWidget(current)
            if current_widget is not None:
                current_widget.set_selected_style(True)

    def frame_index_from_id(self, frame_id):
        if not frame_id:
            return -1
        return next(
            (
                index for index, frame in enumerate(self.frames)
                if frame.get("frame_id") == frame_id
            ),
            -1,
        )

    def selected_library_frame_index(self):
        """현재 보이는 탭과 무관하게 실제 편집/실행 대상을 반환합니다."""
        return self.frame_index_from_id(self.selected_frame_id)

    def select_full_frame_for_editing(self, row):
        """전체 프레임 목록에서 선택한 원본을 실행 대상으로 지정합니다."""
        if row < 0 or row >= len(self.frames):
            return
        frame_id = self.frames[row].get("frame_id")
        self.last_full_frame_id = frame_id
        self.selected_frame_id = frame_id
        self.load_frame_to_ui(row)

    def on_frame_library_tab_changed(self, tab_index):
        """탭 전환 뒤 숨은 즐겨찾기 행이 전체 목록 실행에 섞이지 않게 합니다."""
        if tab_index != 0:
            return

        # Qt가 많은 setItemWidget 행을 숨겼다가 복원할 때 항목은 남아 있어도
        # 화면이 비는 현상이 있어, 전체 탭으로 돌아올 때 행 위젯을 재구성합니다.
        self.rebuild_frame_list_ui()

        row = self.frame_index_from_id(self.last_full_frame_id)
        if row >= 0:
            self.frame_list_ui1.setCurrentRow(row)
            self.frame_list_ui1.scrollToItem(self.frame_list_ui1.item(row))
            self.selected_frame_id = self.last_full_frame_id

    def execute_frame(self):
        row = self.selected_library_frame_index()
        if row < 0: return QMessageBox.warning(self, "경고", "실행할 프레임을 선택하세요!")
        # 단일 실행도 시퀀스와 동일한 200Hz 코사인 보간 경로를
        # 사용합니다. 모터 내부 시간 프로파일 경로를 별도로 사용하지 않습니다.
        self.load_frame_to_ui(row)
        self.apply_selected_frame()

    def apply_selected_frame(self):
        row = self.selected_library_frame_index()
        if row < 0 or row >= len(self.frames):
            QMessageBox.warning(self, "경고", "불러올 프레임을 목록에서 선택하세요.")
            return

        frame = self.frames[row]
        angles = self.normalize_angles(frame.get("angles", {}))
        if not angles:
            QMessageBox.warning(self, "경고", "선택한 프레임에 적용할 각도 데이터가 없습니다.")
            return

        if self.is_playing:
            self.stop_motion_sequence()

        # 저장된 프레임 시간을 그대로 사용합니다. 기존의 최소 1초
        # 제한은 짧은 키프레임을 임의로 느리게 만들어 제거했습니다.
        duration_ms = max(CONTROL_INTERVAL_MS, int(frame.get("time_ms", 500)))
        self.time_spinbox.setValue(int(frame.get("time_ms", 500)))

        target_ids = sorted(set(angles) & set(self.online_joints))
        missing_ids = sorted(set(angles) - set(self.online_joints))
        if self.port_opened and missing_ids:
            return QMessageBox.warning(
                self,
                "프레임 전체 실행 불가",
                "오프라인 모터가 있어 일부 관절만 실행하지 않습니다.\n"
                f"오프라인 ID: {missing_ids}",
            )

        if self.port_opened and target_ids:
            torque_off_ids = [
                j_id for j_id in target_ids
                if not self.torque_btns[j_id].isChecked()
            ]
            if torque_off_ids:
                # 시퀀스 실행과 같이 현재 자세를 Goal로 먼저 맞춘 뒤
                # 전체 토크를 켜서, 일부 관절만 빠진 자세를 막습니다.
                self.set_all_torque_on()
                still_off_ids = [
                    j_id for j_id in target_ids
                    if not self.torque_btns[j_id].isChecked()
                ]
                if still_off_ids:
                    return QMessageBox.warning(
                        self,
                        "토크 ON 실패",
                        "전체 관절을 잠금지 못해 프레임 실행을 중단합니다.\n"
                        f"OFF ID: {still_off_ids}",
                    )

            self.frame_apply_timer.stop()
            self.frame_apply_on_complete = None

            # 목록 선택 시 GUI에는 이미 목표값이 표시될 수 있으므로, 실제 모터
            # 위치를 시작점으로 다시 읽어 갑작스러운 첫 점프를 방지합니다.
            start_angles = {}
            for j_id in target_ids:
                current_angle = self.read_present_angle(j_id)
                if current_angle is None:
                    QMessageBox.warning(self, "통신 에러", f"ID {j_id}의 현재 각도를 읽지 못해 프레임 적용을 중단했습니다.")
                    return
                start_angles[j_id] = current_angle
                self.update_joint_display(j_id, current_angle, update_robot=False)

            # 이 경로는 200Hz 중간각 보간을 사용하므로 직전 시퀀스의
            # time-based 도착시간이 각 중간 Goal마다 다시 시작되지 않게 0으로 초기화합니다.
            if (set(target_ids).issubset(self.time_based_profile_ids)
                    and not self.configure_unlimited_motor_profiles(target_ids)):
                QMessageBox.warning(self, "통신 에러", "프레임 보간용 프로파일 초기화에 실패했습니다.")
                return

            self.frame_apply_start_angles = start_angles
            self.frame_apply_target_angles = {j_id: angles[j_id] for j_id in target_ids}
            self.frame_apply_ids = target_ids
            self.frame_apply_duration = duration_ms / 1000.0
            self.frame_apply_start_time = time.perf_counter()
            self.frame_apply_name = frame["name"]
            self.frame_apply_frame = copy.deepcopy(frame)
            self.frame_apply_frame["start_ms"] = 0
            self.frame_apply_frame["time_ms"] = duration_ms
            self.update_3d_robot()
            self.frame_apply_timer.start()
            print(f"[🔄 프레임 이동 시작] '{frame['name']}' -> {duration_ms}ms 동안 부드럽게 이동")
        else:
            for j_id, angle in angles.items():
                self.update_joint_display(j_id, angle, update_robot=False)
            self.update_3d_robot()
            print(f"[ℹ️ 프레임 불러오기] '{frame['name']}' 각도를 GUI에 적용했습니다. 토크 ON 모터는 없습니다.")

    def step_frame_apply(self):
        if not self.frame_apply_ids:
            self.frame_apply_timer.stop()
            return

        elapsed_sec = time.perf_counter() - self.frame_apply_start_time
        if self.frame_apply_frame is not None:
            progress = self.frame_motion_progress(
                self.frame_apply_frame,
                elapsed_sec * 1000.0,
            )
        else:
            progress = min(1.0, elapsed_sec / self.frame_apply_duration)
        # 시퀀스 실기 재생과 동일한 half-cosine 진행률을 사용합니다.
        intermediate = {
            j_id: self.interpolate_angle_shortest(
                self.frame_apply_start_angles[j_id],
                self.frame_apply_target_angles[j_id],
                progress,
            )
            for j_id in self.frame_apply_ids
        }

        if not self.write_goal_positions(intermediate, target_ids=self.frame_apply_ids):
            self.frame_apply_timer.stop()
            self.frame_apply_ids = []
            self.frame_apply_frame = None
            self.frame_apply_on_complete = None
            if self.robot_sync_enabled:
                self.btn_robot_sync.setChecked(False)
            self.is_playing = False
            self.is_paused = False
            self.update_playback_buttons()
            QMessageBox.warning(self, "통신 에러", "프레임 이동 중 모터 전송에 실패했습니다.")
            return

        for j_id, angle in intermediate.items():
            self.update_joint_display(j_id, angle, update_robot=False)
        self.update_3d_robot()

        if elapsed_sec >= self.frame_apply_duration:
            # 타이머 지터로 종료 tick이 조금 늦어져도, 보간값이 아닌
            # JSON에 저장된 최종 목표각을 전 관절에 정확히 다시 전송합니다.
            final_ids = list(self.frame_apply_ids)
            final_angles = dict(self.frame_apply_target_angles)
            if not self.write_goal_positions(
                final_angles,
                target_ids=final_ids,
                read_feedback=False,
            ):
                self.frame_apply_timer.stop()
                self.frame_apply_ids = []
                self.frame_apply_frame = None
                self.frame_apply_on_complete = None
                return QMessageBox.warning(
                    self, "통신 에러", "프레임 최종 목표각 전송에 실패했습니다."
                )
            for j_id, angle in final_angles.items():
                self.update_joint_display(j_id, angle, update_robot=False)
            self.update_3d_robot()
            self.frame_apply_timer.stop()
            print(
                f"[✅ 프레임 최종각 전송 완료] '{self.frame_apply_name}' "
                f"{len(final_ids)}개 관절 JSON Goal Position 적용"
            )
            self.frame_apply_ids = []
            self.frame_apply_frame = None
            on_complete = self.frame_apply_on_complete
            self.frame_apply_on_complete = None
            if on_complete is not None:
                on_complete()

    def load_frame_to_ui(self, row):
        if row < 0 or row >= len(self.frames): return
        frame = self.frames[row]
        self.selected_frame_id = frame.get("frame_id")
        self.editing_loaded_pose = True
        self.time_spinbox.setValue(int(frame.get("time_ms", 500)))
        for j_id, angle in self.normalize_angles(frame["angles"]).items():
            # 저장 자세 편집은 하드웨어 연결/토크 여부와 무관하게 가능합니다.
            if j_id in self.sliders:
                self.sliders[j_id].setEnabled(True)
            if j_id in self.spinboxes:
                self.spinboxes[j_id].setEnabled(True)
            self.update_joint_display(j_id, angle, update_robot=False)
        self.update_3d_robot()

    def add_frame(self):
        duration = self.time_spinbox.value()
        frame_data = {"frame_id": uuid.uuid4().hex, "name": f"Frame {len(self.frames) + 1}", "time_ms": duration, "angles": self.joints.copy(), "torques": {j: b.isChecked() for j, b in self.torque_btns.items()}, "is_important": False, "lift_early_arrival": True}
        self.frames.append(frame_data)
        item = QListWidgetItem(self.frame_list_ui1)
        custom_widget = FrameItemWidget(frame_data, self)
        item.setSizeHint(custom_widget.sizeHint())
        self.frame_list_ui1.setItemWidget(item, custom_widget)
        self.frame_list_ui1.scrollToBottom()
        self.refresh_library_lists()

    def propagate_library_frame_update(self, source_frame):
        """
        라이브러리 원본의 자세 변경을 현재 타임라인과 저장 시퀀스에 전파합니다.
        시퀀스에서 따로 편집한 start_ms/time_ms는 동작 타이밍이므로 보존합니다.
        """
        source_id = source_frame.get("frame_id")
        source_name = source_frame.get("name")
        same_name_count = sum(
            1 for frame in self.frames if frame.get("name") == source_name
        )
        updated = 0
        collections = [self.motion_sequence]
        collections.extend(
            sequence.get("frames", []) for sequence in self.saved_sequences
        )
        for frames in collections:
            for sequence_frame in frames:
                linked_id = (
                    sequence_frame.get("source_frame_id")
                    or sequence_frame.get("frame_id")
                )
                same_source = bool(source_id) and linked_id == source_id
                # 예전 JSON에는 source_frame_id가 없을 수 있으므로 라이브러리에
                # 같은 이름이 하나뿐일 때만 안전하게 이름으로 연결을 복구합니다.
                safe_legacy_match = (
                    same_name_count == 1
                    and sequence_frame.get("name") == source_name
                )
                if not (same_source or safe_legacy_match):
                    continue

                sequence_frame["frame_id"] = source_id
                sequence_frame["source_frame_id"] = source_id
                sequence_frame["name"] = source_name
                sequence_frame["angles"] = copy.deepcopy(source_frame["angles"])
                sequence_frame["torques"] = copy.deepcopy(source_frame["torques"])
                updated += 1
        return updated

    def update_frame(self):
        row = self.selected_library_frame_index()
        if row < 0: return QMessageBox.warning(self, "경고", "재저장할 프레임을 선택하세요.")
        source_frame = self.frames[row]
        source_frame.setdefault("frame_id", uuid.uuid4().hex)
        source_frame["angles"] = self.joints.copy()
        source_frame["torques"] = {j: b.isChecked() for j, b in self.torque_btns.items()}
        source_frame["time_ms"] = self.time_spinbox.value()

        updated_sequence_frames = self.propagate_library_frame_update(source_frame)
        self.resort_motion_sequence()
        self.refresh_timeline_ui()
        widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(row))
        if widget: widget.label.setText(f"[{self.frames[row]['name']}] {self.frames[row]['time_ms']}ms")
        self.refresh_library_lists()
        message = (
            f"프레임이 재저장되었고 시퀀스 블록 "
            f"{updated_sequence_frames}개에 바로 반영되었습니다."
        )
        QMessageBox.information(self, "재저장", message)

    def rename_frame(self):
        row = self.selected_library_frame_index()
        if row < 0: return
        new_name, ok = QInputDialog.getText(self, '이름 변경', '새 프레임 이름:', text=self.frames[row]["name"])
        if ok and new_name.strip():
            self.frames[row]["name"] = new_name.strip()
            widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(row))
            if widget: widget.label.setText(f"[{new_name.strip()}] {self.frames[row]['time_ms']}ms")
            self.refresh_library_lists()

    def mirror_frame(self):
        # 먼저 전 관절을 복사해 지정되지 않은 모터 값은 완전히 보존합니다.
        mirrored = dict(self.joints)
        for r_id, (l_id, sign) in self.mirror_map.items():
            mirrored[l_id], mirrored[r_id] = self.joints[r_id] * sign, self.joints[l_id] * sign
        for j_id, angle in mirrored.items():
            self.update_joint_display(j_id, angle, update_robot=False)
        self.update_3d_robot()

    def toggle_select_mode(self, checked):
        self.is_select_mode = checked
        self.btn_toggle_select.setStyleSheet("background-color: #d1ecf1;" if checked else "")
        if not checked: self.frame_list_ui1.clearSelection()
        for i in range(self.frame_list_ui1.count()):
            widget = self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(i))
            if widget:
                widget.checkbox.setVisible(checked)
                if not checked: widget.checkbox.setChecked(False)

    def delete_frame(self):
        if self.is_select_mode:
            rows = [i for i in range(self.frame_list_ui1.count()) if self.frame_list_ui1.itemWidget(self.frame_list_ui1.item(i)).checkbox.isChecked()]
            if not rows: return QMessageBox.warning(self, "경고", "삭제할 프레임을 체크해주세요.")
            if QMessageBox.question(self, '확인', "선택 삭제하시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                for idx in sorted(rows, reverse=True):
                    self.frames.pop(idx)
                    self.frame_list_ui1.takeItem(idx)
                self.refresh_library_lists()
        else:
            row = self.selected_library_frame_index()
            if row >= 0 and QMessageBox.question(self, '확인', "삭제하시겠습니까?", QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
                self.frames.pop(row)
                self.frame_list_ui1.takeItem(row)
                self.refresh_library_lists()

    def on_all_selected(self):
        pass

    def select_favorite_frame_for_editing(self, item):
        """즐겨찾기 항목을 원본 프레임 편집 대상으로 선택합니다."""
        frame_idx = item.data(Qt.UserRole)
        if frame_idx is None or not (0 <= frame_idx < len(self.frames)):
            return
        # 숨겨진 전체 목록의 currentRow를 바꾸지 않습니다. 여기서 그 행을
        # 바꾸면 전체 탭으로 복귀한 뒤 화면과 실제 실행 대상이 어긋납니다.
        self.selected_frame_id = self.frames[frame_idx].get("frame_id")
        self.load_frame_to_ui(frame_idx)

    def remove_selected_frame_favorite(self):
        """즐겨찾기 탭에서 선택한 원본 프레임의 별표를 해제합니다."""
        item = self.frame_favorites_ui.currentItem()
        if item is None:
            return QMessageBox.warning(
                self,
                "경고",
                "즐겨찾기에서 해제할 프레임을 선택하세요.",
            )
        frame_idx = item.data(Qt.UserRole)
        if frame_idx is None or not (0 <= frame_idx < len(self.frames)):
            return
        self.frames[frame_idx]["is_important"] = False
        self.refresh_library_lists()

    def toggle_favorite_frames_filter(self, checked):
        """전체 프레임 목록을 즐겨찾기만/전체 표시로 전환합니다."""
        self.show_favorite_frames_only = bool(checked)
        self.btn_favorite_frames_only.setText(
            "★ 즐겨찾기만 보는 중"
            if checked
            else "☆ 즐겨찾기만 보기"
        )
        self.refresh_library_lists()

    def open_frame_editor_from_library_item(self, item):
        """전체/즐겨찾기 목록에서 원본 프레임 편집 위치로 이동합니다."""
        frame_idx = item.data(Qt.UserRole)
        if frame_idx is None or not (0 <= frame_idx < len(self.frames)):
            return
        self.tabs.setCurrentWidget(self.tab_frame)
        self.frame_library_tabs.setCurrentIndex(0)
        self.frame_list_ui1.setCurrentRow(frame_idx)
        self.frame_list_ui1.scrollToItem(self.frame_list_ui1.item(frame_idx))
        self.load_frame_to_ui(frame_idx)

    def delete_selected_saved_sequence(self):
        item = self.sequence_list_ui.currentItem()
        if item is None:
            return QMessageBox.warning(self, "경고", "삭제할 저장 시퀀스를 선택하세요.")
        self.delete_saved_sequence(item.data(Qt.UserRole))

    def rename_selected_saved_sequence(self):
        item = self.sequence_list_ui.currentItem()
        if item is None:
            return QMessageBox.warning(
                self,
                "경고",
                "이름을 변경할 저장 시퀀스를 선택하세요.",
            )
        sequence_idx = item.data(Qt.UserRole)
        if sequence_idx is None or not (0 <= sequence_idx < len(self.saved_sequences)):
            return

        sequence = self.saved_sequences[sequence_idx]
        old_name = str(sequence.get("name", "Sequence"))
        new_name, ok = QInputDialog.getText(
            self,
            "시퀀스 이름 변경",
            "새 시퀀스 이름:",
            text=old_name,
        )
        new_name = new_name.strip()
        if not ok or not new_name or new_name == old_name:
            return
        if any(
            index != sequence_idx and str(saved.get("name", "")) == new_name
            for index, saved in enumerate(self.saved_sequences)
        ):
            return QMessageBox.warning(
                self,
                "이름 중복",
                f"'{new_name}' 이름의 시퀀스가 이미 있습니다.",
            )

        sequence["name"] = new_name
        self.refresh_sequence_list()
        self.sequence_list_ui.setCurrentRow(sequence_idx)
        if sequence.get("sequence_id") == self.loaded_sequence_id:
            self.lbl_loaded_sequence.setText(
                f"현재 타임라인: {new_name} (이름 변경됨)"
            )
        self.update_sequence_save_button()
        self.save_persistent_state()
        QMessageBox.information(
            self,
            "이름 변경 완료",
            f"'{old_name}' → '{new_name}'\n프레임과 시간 데이터는 변경하지 않았습니다.",
        )

    def delete_composer_selected_sequence(self):
        item = self.composer_source_list.currentItem()
        if item is None:
            return QMessageBox.warning(self, "경고", "삭제할 저장 시퀀스를 선택하세요.")
        self.delete_saved_sequence(item.data(Qt.UserRole))

    def delete_saved_sequence(self, sequence_idx):
        if sequence_idx is None or not (0 <= sequence_idx < len(self.saved_sequences)):
            return
        sequence_name = self.saved_sequences[sequence_idx]['name']
        deleted_sequence_id = self.saved_sequences[sequence_idx].get("sequence_id")
        reply = QMessageBox.question(
            self,
            "시퀀스 삭제",
            f"저장된 시퀀스 '{sequence_name}'을(를) 삭제하시겠습니까?\n"
            "실수로 삭제한 경우 Ctrl+Z로 복구할 수 있습니다.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply != QMessageBox.Yes:
            return

        if self.playback_context == "composer":
            self.stop_motion_sequence()
        self.saved_sequences.pop(sequence_idx)
        if deleted_sequence_id == self.loaded_sequence_id:
            self.loaded_sequence_id = None
            self.lbl_loaded_sequence.setText("현재 타임라인: 원본 시퀀스 삭제됨 (새로 저장 가능)")
        self.update_sequence_save_button()

        remaining_entries = []
        for entry in self.sequence_composer_entries:
            entry_idx = entry['sequence_idx']
            if entry_idx == sequence_idx:
                continue
            if entry_idx > sequence_idx:
                entry['sequence_idx'] = entry_idx - 1
            remaining_entries.append(entry)
        self.sequence_composer_entries = remaining_entries
        self.pack_sequence_composer_entries()
        self.refresh_sequence_composer_timeline()
        self.refresh_sequence_list()
        self.save_persistent_state()
        QMessageBox.information(self, "삭제 완료", f"'{sequence_name}' 시퀀스를 삭제했습니다.")

    def refresh_sequence_list(self):
        if not hasattr(self, 'sequence_list_ui'):
            return
        self.sequence_list_ui.clear()
        for idx, seq in enumerate(self.saved_sequences):
            item = QListWidgetItem(
                f"[{idx+1}] {seq['name']} ({len(seq['frames'])} 프레임 / "
                f"{seq.get('repeat_count', 1)}회 / {seq.get('playback_speed', 1.0):.2f}x)"
            )
            item.setData(Qt.UserRole, idx)
            self.sequence_list_ui.addItem(item)
        self.refresh_composer_source_list()

    def refresh_library_lists(self):
        # 1번 탭의 편집 목록과 2번 탭의 전체 프레임 목록은 같은
        # self.frames를 사용합니다. 어느 저장/복구 경로에서든 한쪽 UI만
        # 갱신된 경우를 감지해 편집 목록을 즉시 다시 맞춥니다.
        if hasattr(self, "frame_list_ui1"):
            visible_ids = []
            for row in range(self.frame_list_ui1.count()):
                item = self.frame_list_ui1.item(row)
                widget = self.frame_list_ui1.itemWidget(item)
                visible_ids.append(
                    widget.frame_data.get("frame_id")
                    if widget is not None else None
                )
            expected_ids = [frame.get("frame_id") for frame in self.frames]
            if visible_ids != expected_ids:
                selected_id = self.last_full_frame_id
                previous_count = len(visible_ids)
                self.frame_list_ui1.clear()
                restored_row = -1
                for index, frame_data in enumerate(self.frames):
                    item = QListWidgetItem(self.frame_list_ui1)
                    custom_widget = FrameItemWidget(frame_data, self)
                    item.setSizeHint(custom_widget.sizeHint())
                    self.frame_list_ui1.setItemWidget(item, custom_widget)
                    if frame_data.get("frame_id") == selected_id:
                        restored_row = index
                if restored_row >= 0:
                    self.frame_list_ui1.setCurrentRow(restored_row)
                    self.frame_list_ui1.scrollToItem(
                        self.frame_list_ui1.item(restored_row)
                    )
                elif len(self.frames) > previous_count:
                    self.frame_list_ui1.scrollToBottom()

        if hasattr(self, "frame_favorites_ui"):
            self.frame_favorites_ui.clear()
            for frame_idx, frame in enumerate(self.frames):
                if not frame.get("is_important", False):
                    continue
                favorite_item = QListWidgetItem(
                    f"★ [{frame['name']}] {frame['time_ms']}ms"
                )
                favorite_item.setData(Qt.UserRole, frame_idx)
                self.frame_favorites_ui.addItem(favorite_item)
            if hasattr(self, "frame_library_tabs"):
                self.frame_library_tabs.setTabText(
                    0,
                    f"전체 프레임 ({len(self.frames)})",
                )
                self.frame_library_tabs.setTabText(
                    1,
                    f"★ 즐겨찾기 ({self.frame_favorites_ui.count()})",
                )

        self.frame_list_all.clear()
        favorite_count = sum(
            bool(frame.get('is_important', False)) for frame in self.frames
        )
        for idx, frame in enumerate(self.frames):
            if (
                self.show_favorite_frames_only
                and not frame.get('is_important', False)
            ):
                continue
            display_text = f"{'★ ' if frame.get('is_important', False) else ''}[{frame['name']}] {frame['time_ms']}ms"
            item_all = QListWidgetItem(display_text)
            item_all.setData(Qt.UserRole, idx) 
            self.frame_list_all.addItem(item_all)
        if hasattr(self, "lbl_all_frames"):
            visible_name = (
                "즐겨찾기 프레임"
                if self.show_favorite_frames_only
                else "전체 프레임"
            )
            self.lbl_all_frames.setText(
                f"📁 {visible_name} "
                f"({self.frame_list_all.count()}개 표시 / ★ {favorite_count}개)"
            )
        self.refresh_sequence_list()
        self.save_persistent_state()
        self.save_persistent_state()

    def export_motion_json(self):
        if not self.motion_sequence: return
        self.repair_legacy_sequence_timing(self.motion_sequence)
        fileName, _ = QFileDialog.getSaveFileName(self, "모션 저장", "jetson_motion_data.json", "JSON Files (*.json)")
        if fileName:
            export_data = {
                "max_seq_ms": self.max_seq_ms,
                "repeat_count": self.spin_motion_repeat.value(),
                "playback_speed": self.spin_motion_speed.value(),
                "frames": self.motion_sequence
            }
            with open(fileName, 'w', encoding='utf-8') as f:
                json.dump(export_data, f, indent=4, ensure_ascii=False)
            QMessageBox.information(self, "성공", "데이터 추출 완료!")

    def export_all_motions_json(self):
        if not self.saved_sequences:
            return QMessageBox.warning(self, "저장 불가", "저장된 시퀀스가 없습니다.")

        names = [str(sequence.get("name", "")).strip() for sequence in self.saved_sequences]
        normalized_names = [name.casefold() for name in names]
        if any(not name for name in names) or len(set(normalized_names)) != len(names):
            return QMessageBox.warning(
                self, "저장 불가", "빈 이름 또는 중복 이름의 시퀀스가 있습니다."
            )

        for sequence in self.saved_sequences:
            self.repair_legacy_sequence_timing(sequence.get("frames", []))

        file_name, _ = QFileDialog.getSaveFileName(
            self, "전체 모션 JSON 저장", "robot_motions.json", "JSON Files (*.json)"
        )
        if not file_name:
            return
        export_data = {
            "version": 1,
            "motions": [
                {
                    "name": name,
                    "max_seq_ms": sequence.get("max_seq_ms", 5000),
                    "repeat_count": sequence.get("repeat_count", 1),
                    "playback_speed": sequence.get("playback_speed", 1.0),
                    "repeatable": bool(sequence.get("repeatable", True)),
                    "start_pose": (
                        sequence.get("frames", [{}])[0].get("name", "")
                        if sequence.get("frames") else ""
                    ),
                    "end_pose": (
                        sequence.get("frames", [{}])[-1].get("name", "")
                        if sequence.get("frames") else ""
                    ),
                    "completion": {
                        "position_tolerance_deg": 2.0,
                        "settle_duration_ms": 80,
                        "settle_timeout_ms": 3000,
                    },
                    "frames": sequence.get("frames", []),
                }
                for name, sequence in zip(names, self.saved_sequences)
            ],
        }
        try:
            with open(file_name, "w", encoding="utf-8") as file:
                json.dump(export_data, file, indent=4, ensure_ascii=False)
        except (OSError, TypeError, ValueError) as exc:
            return QMessageBox.warning(self, "저장 실패", str(exc))
        QMessageBox.information(
            self, "저장 완료", f"시퀀스 {len(self.saved_sequences)}개를 내보냈습니다."
        )

if __name__ == '__main__':
    if hasattr(Qt, 'AA_EnableHighDpiScaling'): QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    if hasattr(Qt, 'AA_UseHighDpiPixmaps'): QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    app = QApplication(sys.argv)
    app.setStyle("Fusion")
    font = app.font()
    font.setPointSize(12) 
    app.setFont(font)
    signal.signal(signal.SIGINT, signal.SIG_DFL)
    ex = SDKMotionEditor()
    ex.show()
    sys.exit(app.exec_())
