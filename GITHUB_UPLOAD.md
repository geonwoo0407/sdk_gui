# GitHub Upload Guide

이 폴더는 현재 `.git` 디렉터리가 비어 있어 Git 저장소로 인식되지 않습니다.

GitHub에 올릴 때는 먼저 정상 Git 저장소를 초기화해야 합니다.

## 1. 저장소 초기화

기존 빈 `.git` 디렉터리를 정리한 뒤 초기화합니다.

```bash
rm -rf .git
git init
```

## 2. 포함/제외 확인

```bash
git status --short
```

다음 파일은 `.gitignore`로 제외됩니다.

- `__pycache__/`
- `sdk_gui_state.json`
- `step_patched.urdf`
- 가상환경 디렉터리
- 로그/임시 파일

## 3. 첫 커밋

```bash
git add .
git commit -m "Initial SDK motion GUI"
```

## 4. GitHub 원격 저장소 연결

GitHub에서 빈 repository를 만든 뒤 원격 주소를 연결합니다.

```bash
git remote add origin https://github.com/<user>/<repo>.git
git branch -M main
git push -u origin main
```

## 주의

- `sdk_gui_state.json`은 개인 작업 상태라 업로드하지 않습니다.
- `meshes/*.STL`은 로봇 모델 자산입니다. 현재 전체 용량은 GitHub 일반 저장소에 올릴 수 있는 수준입니다.
- 실제 로봇 구동 코드는 하드웨어에 직접 명령을 보내므로 공개 저장소 README의 주의사항을 유지하세요.
