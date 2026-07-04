# IRC STEP SDK Motion GUI

PyQt5 기반 DYNAMIXEL 휴머노이드 모션 편집기입니다.

## 주요 기능

- 23개 DYNAMIXEL 관절 제어
- 전체/개별 토크 ON/OFF
- 온라인 모터 재탐색
- 실제 로봇 관절값 읽기
- 단일 프레임 저장/수정/삭제/좌우반전
- 프레임을 타임라인에 배치해 모션 시퀀스 구성
- 시퀀스 저장/자동 복원
- 실제 로봇 시퀀스 구동
- Jetson용 JSON 내보내기

## 실행 환경

- Python 3.9 이상 권장
- Linux 환경
- ROBOTIS DYNAMIXEL SDK
- U2D2 또는 호환 USB 시리얼 장치
- 기본 포트 탐색: `/dev/ttyUSB0` ~ `/dev/ttyUSB3`
- 기본 baudrate: `4000000`

## 설치

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## 실행

```bash
python3 sdk_gui.py
```

시리얼 권한 문제가 있으면 사용자를 `dialout` 그룹에 추가하거나 포트 권한을 확인하세요.

```bash
sudo usermod -aG dialout "$USER"
```

그룹 변경 후에는 로그아웃/재로그인이 필요합니다.

## 자동 저장

앱은 프레임, 저장된 시퀀스, 현재 타임라인을 `sdk_gui_state.json`에 자동 저장합니다.

이 파일은 개인 작업 상태이므로 Git에는 포함하지 않습니다.

## 주의

- 실제 로봇 구동 전 주변 공간과 전원 상태를 확인하세요.
- `로봇 실제 구동`은 실행 전에 온라인 모터와 토크 상태를 다시 확인합니다.
- 모터 위치 변환은 DYNAMIXEL absolute position 기준 `0~4095`, center `2048`, `4096 step/rev`를 사용합니다.

## 포함 파일

- `sdk_gui.py`: 메인 GUI 애플리케이션
- `requirements.txt`: Python 의존성
- `step.urdf`, `meshes/`: 기존 로봇 모델 자산
- `import matplotlib.ini`: 모터 ID 맵 이미지를 생성하는 보조 Python 스크립트
