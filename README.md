```bash
cd ~/insite
cat > README.md << 'GUIDE'
# AWR Robot 프로젝트 팀원 가이드북

## 시작 전 필수 확인사항

- 라즈베리파이가 켜져 있어야 합니다
- 본인 노트북이 아래 WiFi 중 하나에 연결되어 있어야 합니다

| 장소 | WiFi 이름 | 비밀번호 |
|------|-----------|---------|
| 연구실 | `326_AP1` | `43024302a!` |
| 실험실 | `스마트팩토리608` | `smart608` |
| 실험실 5GHz | `스마트팩토리608_5G` | `smart608` |
| 이동 중 | `JB` (한지범 핫스팟) | `11112222` |

---

## Part 1. 윈도우 PC 초기 환경 설정 (최초 1회만)

### 1-1. VSCode 설치

1. 브라우저에서 아래 주소 접속합니다
```
https://code.visualstudio.com
```
2. **Download for Windows** 버튼 클릭
3. 다운로드된 설치 파일 실행
4. 설치 중 **"Add to PATH"** 옵션 반드시 체크

### 1-2. VSCode Remote-SSH 확장 설치

1. VSCode 실행
2. 왼쪽 사이드바에서 네모 4개 아이콘(Extensions) 클릭
3. 검색창에 `Remote - SSH` 입력
4. **Microsoft** 제공 항목 선택 후 **Install** 클릭

### 1-3. SSH 설정 파일 등록

1. VSCode에서 `Ctrl+Shift+P` 누름
2. `Remote-SSH: Open SSH Configuration File` 입력 후 Enter
3. `C:\Users\사용자명\.ssh\config` 선택
4. 아래 내용을 그대로 복사해서 붙여넣기 후 저장(`Ctrl+S`)

```
Host awr-pi
    HostName pi.local
    User pi
    Port 22
```

### 1-4. GitHub 계정 설정

1. 브라우저에서 `https://github.com` 접속
2. 계정이 없으면 Sign up으로 가입
3. 한지범에게 본인 GitHub 계정명 알려주기 (Collaborator 초대 받아야 함)
4. 이메일로 초대장 오면 수락 클릭

### 1-5. Git 설치

1. 브라우저에서 아래 주소 접속
```
https://git-scm.com/download/win
```
2. **64-bit Git for Windows Setup** 다운로드 후 설치
3. 설치 중 모든 옵션 기본값 유지

---

## Part 2. 라즈베리파이 접속 방법

### 방법 A — VSCode로 접속 (권장)

1. VSCode 실행
2. `Ctrl+Shift+P` 누름
3. `Remote-SSH: Connect to Host` 입력 후 Enter
4. `awr-pi` 선택
5. 비밀번호 입력: `(한지범에게 문의)`
6. 새 VSCode 창이 열리면 접속 완료
7. 상단 메뉴 **Terminal → New Terminal** 클릭하면 Pi 터미널 사용 가능

### 방법 B — PowerShell로 접속

1. 시작 버튼 → `PowerShell` 검색 → 실행
2. 아래 명령어 입력

```powershell
ssh pi@pi.local
```

3. 처음 접속 시 아래 메시지가 뜨면 `yes` 입력

```
Are you sure you want to continue connecting (yes/no)?
```

4. 비밀번호 입력 후 Enter

### 접속이 안 될 때

`pi.local` 로 안 되면 IP 주소로 시도합니다.

```powershell
ssh pi@192.168.0.50
```

---

## Part 3. 접속 후 기본 설정 (매번 실험 시)

SSH 접속 후 아래 두 줄을 반드시 먼저 실행합니다.

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
```

실행 후 프롬프트가 아래처럼 바뀌면 준비 완료입니다.

```
(.venv) pi@pi:~/insite $
```

---

## Part 4. 대시보드 실행 (가장 중요)

### 4-1. 대시보드 실행

Pi 터미널에서 아래 명령어 실행합니다.

```bash
cd ~/insite
source .venv/bin/activate
python app/dashboard.py
```

아래와 같이 출력되면 정상입니다.

```
대시보드 시작: http://192.168.0.50:5000
 * Running on http://192.168.0.50:5000
```

### 4-2. 브라우저에서 접속

Pi와 같은 WiFi에 연결된 노트북 브라우저에서 아래 주소 입력합니다.

```
http://192.168.0.50:5000
```

### 4-3. 대시보드 종료

Pi 터미널에서 `Ctrl+C` 를 누릅니다.

---

## Part 5. 최신 코드 받기

```bash
cd ~/insite
git pull origin master
```

---

## Part 6. 센서 개별 테스트

가상환경 활성화 먼저 확인합니다.

```bash
source ~/insite/.venv/bin/activate
cd ~/insite
```

| 센서 | 실행 명령어 | 종료 |
|------|------------|------|
| 초음파 | `python tests/test_ultra.py` | Ctrl+C |
| 자이로/가속도 | `python tests/test_mpu6050.py` | Ctrl+C |
| 로드셀 | `python tests/test_hx711.py` | Ctrl+C |
| 카메라 | `python tests/test_camera.py` | 자동 |
| LED | `python tests/test_led.py` | 자동 |
| 모터 (들어올린 상태에서) | `python tests/test_motor.py` | 자동 |

---

## Part 7. 코드 수정 및 GitHub 업로드

### 7-1. 처음 한 번만 — Git 사용자 설정

```bash
git config user.email "본인이메일@gmail.com"
git config user.name "본인GitHub계정명"
```

### 7-2. 코드 수정 후 업로드

```bash
cd ~/insite
git add .
git commit -m "수정 내용 간단히 설명"
git push
```

---

## Part 8. 라즈베리파이 안전 종료

실험이 끝나면 반드시 아래 명령어로 종료 후 전원을 뽑습니다.

```bash
sudo shutdown -h now
```

초록 LED가 꺼진 후 전원을 뽑습니다.

---

## 문제 발생 시 확인사항

| 증상 | 해결 방법 |
|------|-----------|
| `pi.local` 접속 안 됨 | `192.168.0.50` 으로 시도 |
| SSH 접속 후 `(.venv)` 안 보임 | `source ~/insite/.venv/bin/activate` 실행 |
| 대시보드 브라우저 접속 안 됨 | Pi에서 `python app/dashboard.py` 실행됐는지 확인 |
| 센서값 이상 | 해당 센서 개별 테스트 실행 |
| 알 수 없는 오류 | 한지범에게 문의 |
GUIDE