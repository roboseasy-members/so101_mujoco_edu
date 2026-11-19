# so102_mujoco_edu — SO-102(7축) MuJoCo 시뮬레이션과 디지털 트윈

SO-102 로봇팔을 MuJoCo 시뮬레이터로 움직이고, 실물 로봇과 연동해 보는 실습 코드입니다.

- **SO-102** = SO-101 손목에 **좌우 회전축(5축) 하나를 더한** 7축 팔
- 시뮬레이션 모델은 SO-102 URDF(로봇 설계도 파일)로 만들어 레포에 넣어 두었습니다 (`src/mujoco_sim/so102/`)
- 목표는 **관절 움직임**과 **디지털 트윈** 개념을 익히는 것

| 실습 | 방향 | 로봇 필요 |
|---|---|---|
| 1 | 시뮬 → 시뮬: 슬라이더로 관절 움직이기 | ✗ |
| 2 | 실기 → 시뮬: 디지털 트윈 | ✓ |
| 3 | 시뮬 → 실기: 시뮬로 로봇 구동 | ✓ |

> 시뮬레이터 = 물리 법칙을 계산해 가상의 로봇을 움직이는 프로그램.
> 코드 시험 · 위험한 동작 미리 보기 · 데이터 만들기에 씁니다.

---

## 설치

lerobot 0.6.0(Python 3.12 이상)과 SO-102 플러그인([roboseasy/so-102](https://github.com/roboseasy/so-102)) 기준입니다.

```bash
conda activate lerobot_060          # 새 터미널마다 다시 켭니다
cd ~/workspace
git clone -b so102_7dof https://github.com/roboseasy-members/so101_mujoco_edu.git so102_mujoco_edu
cd so102_mujoco_edu
pip install -r requirements.txt     # lerobot[feetech]==0.6.0, mujoco==3.12.0, SO-102 플러그인
```

설치 확인:

```bash
pip list | grep mujoco    # mujoco 3.12.0
pip list | grep so102     # lerobot_robot_so102
```

> SO-102 플러그인을 이미 `so-102` 폴더에서 `pip install -e .`로 설치했다면
> `pip install mujoco==3.12.0`만 해도 됩니다.

**아래 명령은 모두 레포 폴더(`~/workspace/so102_mujoco_edu`)에서 실행합니다.**

---

## 관절 7개

| 축 | 관절 이름 | 움직임 | 범위 rad (deg) |
|---|---|---|---|
| 1 | shoulder_pan | 팔 전체 좌우 | ±1.92 (±110°) |
| 2 | shoulder_lift | 어깨 앞뒤 | ±1.75 (±100°) |
| 3 | elbow_flex | 팔꿈치 굽힘 | ±1.69 (±97°) |
| 4 | wrist_flex | 손목 위아래 | -1.43 ~ 1.15 (-82° ~ 66°) |
| 5 | **wrist_yaw** | **손목 좌우 (SO-102 신규)** | -1.47 ~ 1.43 (-84° ~ 82°) |
| 6 | wrist_roll | 그리퍼 제자리 회전 | -2.74 ~ 2.84 (-157° ~ 163°) |
| 7 | gripper | 집게 | -0.175 ~ 1.75 (-10° ~ 100°) |

- 시뮬 슬라이더 숫자는 **라디안**, 실기 LeRobot 값은 **도(°)** 입니다 (1.92 rad × 180/π = 110°)
- 모든 슬라이더 0 = 캘리브레이션 때 맞춘 "가동범위 중앙" 자세

---

## 실습 1. 시뮬 → 시뮬: 슬라이더로 관절 움직이기

```bash
python -m mujoco.viewer --mjcf src/mujoco_sim/so102/scene.xml
```

- 마우스: **왼쪽 드래그** 회전 · **오른쪽 드래그** 이동 · **휠** 확대/축소
- 오른쪽 **Control** 패널을 펼치면 슬라이더 7개(1축~7축)가 있습니다
- 1축부터 7축까지 하나씩 끝까지 움직여 보세요. 특히 **5축**은 SO-101에 없던 축입니다

> 생각해 보기: SO-101에서 손을 옆으로 돌리려면 1축으로 팔 전체를 돌려야 합니다.
> 그러면 손의 위치도 함께 바뀝니다. SO-102의 5축은 무엇이 다를까요?

종료: 터미널에서 `Ctrl` + `C` (안 되면 `Ctrl` + `Z`)

---

## 실습 2. 실기 → 시뮬: 디지털 트윈

실물 로봇팔의 관절 각도를 1초에 30번 읽어 MuJoCo 모델에 그대로 옮깁니다.
필요한 변환은 두 가지뿐입니다 — 도(°) → 라디안, 그리퍼 0~100 → 모델 관절 범위.

```bash
python -m src.mujoco_sim.digital_twin --port /dev/ttyACM0 --id so102_leader_mujoco
```

- `--id`는 캘리브레이션 때 쓴 이름입니다 (공백·한글 없이). 캘리브레이션 확인 메시지가 나오면 `ENTER`
- ⚠ 실행하면 **토크가 풀립니다.** 팔을 손으로 잡거나 눕혀 두고 실행하세요
- 손으로 팔을 움직이면 화면 속 로봇이 따라옵니다. **손목을 좌우로 꺾어 5축이 따라오는지** 꼭 확인하세요
- 이때 뷰어 슬라이더가 안 움직이는 것은 정상입니다 (실기가 1초에 30번 자세를 덮어씀)

종료: `Ctrl` + `C`

---

## 실습 3. 시뮬 → 실기: 시뮬로 로봇 구동

이번엔 시뮬이 명령을 냅니다. 슬라이더를 움직이면 실물 로봇이 따라 움직입니다.

```bash
python -m src.mujoco_sim.sim_to_robot --port /dev/ttyACM1 --id so102_follower_mujoco --max_step_deg 5
```

- 연결하면 토크가 켜지고, 시뮬이 실기 현재 자세로 맞춰지므로 팔이 튀지 않습니다
- `--max_step_deg`: 한 번에 움직일 수 있는 각도 제한. **처음엔 1~5도**
- 7개 관절을 **하나씩, 조금씩**. 먼저 실기가 같은 방향으로 도는지 확인하고, 처음 보는 5축은 천천히
- 슬라이더를 +로 올렸는데 실기가 반대로 돌면 즉시 종료하고 강사에게 알립니다

> ⚠ **안전**
> - 팔 주위와 그리퍼 안을 비우고, 손은 `Ctrl` + `C` 위에 올려 두세요
> - 종료(`Ctrl` + `C` 또는 뷰어 창 닫기) 시 **토크가 풀리므로 로봇을 손으로 받쳐 주고** 종료하세요
> - 그래도 위험하면 전원을 차단하세요

---

## 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `--port` | (필수) | 로봇팔 시리얼 포트 (`lerobot-find-port`로 확인) |
| `--id` | (필수) | 캘리브레이션에 쓴 로봇 id |
| `--scene` | 동봉 `scene.xml` | 다른 MJCF scene을 쓸 때 지정 |
| `--max_step_deg` | `3.0` | (sim_to_robot 전용) 전송 1회당 관절 이동량 상한(deg) |

## 파일 구성

| 파일 | 역할 |
|------|------|
| `src/mujoco_sim/so102/` | SO-102 MJCF 모델·메시 |
| `src/mujoco_sim/sim_bridge.py` | 실기(deg·0~100) ↔ 시뮬(rad) 단위 변환, 관절 목록 |
| `src/mujoco_sim/digital_twin.py` | 실습 2: 실기 → 시뮬 |
| `src/mujoco_sim/sim_to_robot.py` | 실습 3: 시뮬 → 실기 (이동량 제한 포함) |
| `tools/build_so102_mjcf.py` | so-102 URDF → `so102.xml` 생성 (모델을 다시 만들 때만) |
| `tests/` | 실기 없이 도는 테스트 (`python -m pytest tests`) |

## 자주 겪는 문제

- **`No module named 'src'`** — 레포 폴더가 아닌 곳에서 실행했습니다. `cd ~/workspace/so102_mujoco_edu` 후 다시 실행
- **`No module named 'lerobot_robot_so102'`** — SO-102 플러그인이 없습니다. `pip install -r requirements.txt`
- **포트 권한 오류** — `sudo usermod -aG dialout $USER` 후 재로그인 (임시: `sudo chmod 666 /dev/ttyACM0`)
- **캘리브레이션 파일 없음** — `--id`가 캘리브레이션 때 쓴 값과 다릅니다. 같은 id로 `lerobot-calibrate`를 다시 실행
- **시뮬과 실기의 관절 방향·자세가 다름** — `src/mujoco_sim/so102/README.md`를 참고해 `tools/build_so102_mjcf.py`로 모델을 다시 생성
