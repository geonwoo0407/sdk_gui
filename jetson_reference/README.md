# 젯슨 모션 비교용 소스

추가 전체 조사: [FULL_REPOSITORY_AUDIT.md](FULL_REPOSITORY_AUDIT.md). 이제 지정 브랜치의 197개 파일 전체가 IRC_vision/에 있으며 최초 선별 다운로드 이후 추가한 파일은 FULL_AUDIT_MANIFEST.json에 기록했다.

출처: https://github.com/dbwldus0220-sys/IRC_vision/tree/agent/production-integration-20260912
커밋: 45fa1ddc136ff3af0d9ae355d4fbeeb45e5c248f

IRC_vision/에는 모션 실행기 패키지(관련 테스트 포함), 실제 로봇 launch, 실행용 모션 JSON, alias 설정, SDK 연결 문서와 GUI JSON 변환 도구를 선별 복사했다. 비전 모델과 영상 처리 코드는 제외했다. 파일 목록과 SHA256은 IMPORT_MANIFEST.json에 기록했다. 기존 GUI/SDK 파일은 수정하지 않았다.

## 확인된 한계

- 이 브랜치의 CMakeLists.txt는 ROBOT_MOTION_SDK_DIR로 외부 SDK를 연결한다. 실제 RobotMotionPlayer/Dynamixel 구현은 이 저장소에 없다.
- test/fixtures/fake_robot_motion_sdk는 테스트용 대역이며 실제 젯슨 SDK로 취급하면 안 된다.
- production factory는 DynamixelMotionHardwareConfig 및 시작 자세 관련 API를 사용한다. 현재 작업 폴더의 SDK와 동일 버전이라고 가정할 수 없다.
- 저장소의 실행용 JSON과 launch가 실제 젯슨에서 사용 중인 버전인지는 아직 확인하지 않았다.

## 젯슨에서 추가로 필요한 자료

1. 실제 빌드 시 ROBOT_MOTION_SDK_DIR로 지정한 SDK 소스 폴더 전체.
2. build/irc_step_motion_executor/CMakeCache.txt의 ROBOT_MOTION_SDK_DIR 항목(실제 빌드 경로가 다르면 해당 경로).
3. 실제 실행 명령어, 사용한 JSON 경로/파일, 문제가 발생한 모션 이름과 실행 로그.

SDK 경로 확인 예시(젯슨 ROS 작업공간에서):

```bash
rg '^ROBOT_MOTION_SDK_DIR:' build/irc_step_motion_executor/CMakeCache.txt
```

복사 파일 해시와 JSON 구문을 확인했다. ROS 빌드 및 실제 하드웨어 실행은 수행하지 않았다.
