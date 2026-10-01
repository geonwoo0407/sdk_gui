이 묶음은 2026-09-30의 최신 로컬 작업 파일과 구현 비교 자료입니다. 먼저 `REPLY_TO_IRC_VISION.md`를 읽으세요. 동봉 GUI와 C++의 동작은 아직 다르며, C++ API 일부도 ROS 참조 프로젝트의 기대와 다릅니다. `cpp_sdk/README.md`와 `cpp_sdk/SDK_HANDOFF.md`는 원본 문서를 그대로 보존한 것이며, 구형 경로 설명은 이번 답변서의 실제 호출 경로 비교와 함께 읽어야 합니다.

압축을 해제한 최상위 디렉터리에서 무결성을 확인합니다.

```bash
sha256sum -c SHA256SUMS
```

GUI 실행에 필요한 관측 환경은 Python 3.10.12, PyQt5 5.15.6(Qt 라이브러리 패키지 5.15.3), dynamixel-sdk 4.0.5, pyserial 3.5, numpy 1.21.5입니다. `gui/requirements-observed.txt`는 이 PC에서 확인한 Python 패키지 버전 목록이며 다른 OS·Python에서의 설치를 검증한 lockfile은 아닙니다. Python 패키지 버전과 별도 설치된 ROBOTIS C++ 라이브러리 버전을 혼동하지 마세요. C++ ROBOTIS 라이브러리의 버전 번호는 확인하지 못했으며 사용한 파일의 해시만 `ENVIRONMENT.json`에 기록했습니다.

하드웨어 없는 Python 검증은 다음 명령으로 합니다.

```bash
cd gui
QT_QPA_PLATFORM=offscreen python3 -m unittest test_sdk_gui_composer_timeline test_sdk_gui_timed_playback test_sdk_gui_precision test_diagnose_mx_bus -q
```

실제 GUI 실행 명령은 GUI 디렉터리에서 `python3 sdk_gui.py`입니다. **GUI 생성자는 USB 포트를 탐색하며 연결되면 토크 OFF와 Drive Mode 준비 명령을 보낼 수 있으므로, 단순 소프트웨어 검사에는 위 테스트 명령을 사용하세요.** 현재 버전에는 별도의 오프라인 실행 CLI가 없습니다. `step.urdf`가 없어 FK 계산 모델은 로드되지 않으며 3D 표시는 비활성화되어 있습니다.

C++는 CMake 3.16 이상, C++20 컴파일러, Eigen3, Threads, 외부 ROBOTIS DYNAMIXEL SDK 헤더·라이브러리를 사용합니다. 이 PC에서 확인한 도구는 CMake 3.22.1, GCC 11.4.0, Eigen 3.4.0입니다. 외부 ROBOTIS SDK는 전달본에 포함되지 않습니다.

묶음 최상위에서 다음 명령으로 빌드·테스트합니다. 테스트의 assert가 활성화되도록 Debug를 지정합니다.

```bash
cmake -S cpp_sdk -B build -DCMAKE_BUILD_TYPE=Debug
cmake --build build -j2
ctest --test-dir build --output-on-failure
```

외부 SDK가 자동 검색되지 않으면 include 디렉터리와 대상 아키텍처에 맞는 라이브러리 경로를 명시합니다.

```bash
cmake -S cpp_sdk -B build -DCMAKE_BUILD_TYPE=Debug \
  -DDYNAMIXEL_SDK_INCLUDE_DIR=/path/to/include \
  -DDYNAMIXEL_SDK_LIBRARY=/path/to/target-architecture-sdk-library.so
```

현재 CMake 자동 탐색 이름은 `dxl_x64_cpp` 또는 `dynamixel_sdk`입니다. Jetson에서는 그 장치용 라이브러리의 실제 파일 경로를 지정해야 할 수 있습니다. x86_64에서 빌드한 바이너리를 Jetson용으로 사용하면 안 됩니다. `robot_control` 및 `irc_robot` 타깃이 생겼는지 확인하세요. 외부 SDK가 없으면 core 테스트만 빌드될 수 있습니다.

`ctest`의 세 테스트는 mock/core 경로이며 모터를 열지 않습니다. 반면 `irc_robot`이나 실제 하드웨어 생성자를 사용하는 코드는 포트를 열고 Operating Mode·PID·토크를 변경할 수 있습니다. 이번 전달 검증에서는 이 실행 파일을 실행하지 않았습니다.

API와 JSON 로더 검사를 재현하려면 묶음 최상위에서 다음을 실행합니다.

```bash
c++ -std=c++20 -Icpp_sdk verification/api_probe.cpp -o build/api_probe
./build/api_probe
c++ -std=c++20 -Icpp_sdk verification/catalog_probe.cpp build/libmotion_core.a -o build/catalog_probe
./build/catalog_probe cpp_sdk/robot_motions.json
./build/catalog_probe data/robot_motions_from_gui_checked.json
```

최신 GUI 상태를 비교할 데이터는 `data/robot_motions_from_gui_checked.json`입니다. `cpp_sdk/robot_motions.json`은 작업 폴더에 있던 원본 export이며, 한 모션이 최신 GUI와 달라 원본과 차이를 함께 보존했습니다. GUI 데이터는 프레임 691개, 저장 시퀀스 405개, 체크된 배포 시퀀스 85개입니다. 실제 제어 코드 및 원본 상태 파일은 이번 자료 작성 과정에서 변경하지 않았습니다.

`SOURCE_MANIFEST.json`에 스냅샷 시각, 원본 경로·해시, git 기준 커밋과 작업 트리 변경 목록이 있습니다. `verification/`에는 이 전달본에서 실행한 검사 로그가 있습니다. ROS 2 프로젝트 전체 및 외부 SDK 자체를 vendoring한 패키지는 아니며, 원래의 GUI/Python 소스와 이 프로젝트 소유 C++ SDK 소스 전체를 제공합니다.
