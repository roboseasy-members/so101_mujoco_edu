# so101_mujoco_edu — SO-101 MuJoCo 시뮬레이션 실습

실기 SO-101과 MuJoCo 시뮬레이션을 잇는 교육 레포입니다.
SO-101 시뮬레이션 모델이 레포에 동봉돼 있어(`src/mujoco_sim/so101/`,
출처·라이선스는 그 안의 README 참조) **git clone과 pip 설치만으로
모든 실습이 됩니다.**

`dual_a-ba_edu` 레포의 `src/mujoco_sim/` 모듈을 분리한 것으로,
lerobot 0.6.0 기준으로 동작합니다.

## 설치

Python 3.12 이상이 필요합니다(lerobot 0.6.0 요구). venv나 conda 같은
가상환경을 씁니다.

```bash
git clone <레포 URL> && cd so101_mujoco_edu
python -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt   # lerobot[feetech]==0.6.0, mujoco==3.12.0
```

`requirements.txt`에는 lerobot 0.6.0(Feetech 모터 SDK extra)과 mujoco가
들어 있습니다. lerobot이 torch를 함께 설치하므로 첫 설치는 시간이 걸립니다.

## 실기 준비 (실습 2·3에만 필요)

실습 2·3은 캘리브레이션된 SO-101 팔로워암이 필요합니다. 아직 안 했다면
lerobot CLI로 포트 확인과 캘리브레이션을 먼저 합니다.

```bash
lerobot-find-port   # USB를 뽑았다 꽂아 팔로워암 포트 확인 (예: /dev/ttyACM0)
lerobot-calibrate --robot.type=so101_follower \
    --robot.port=/dev/ttyACM0 --robot.id=my_follower
```

여기서 쓴 `--robot.port`와 `--robot.id`를 아래 실습의 `--port`·`--id`에
그대로 넣습니다. 캘리브레이션 파일은 `~/.cache/huggingface/lerobot/calibration/`
아래에 `robots/so_follower/{id}.json`으로 저장됩니다.

## 실습

세 방향이고, 순서대로 진행합니다. 아래 명령은 모두 **레포 루트**에서
실행합니다.

### 1. 시뮬 → 시뮬 (로봇 불필요)

```bash
python -m mujoco.viewer --mjcf src/mujoco_sim/so101/scene.xml
```

뷰어 오른쪽 Control 슬라이더 6개로 관절을 움직여 봅니다.
순기구학(FK)을 눈으로 체감하는 단계입니다.

### 2. 실기 → 시뮬 (디지털 트윈)

```bash
python -m src.mujoco_sim.digital_twin --port /dev/ttyACM0 --id my_follower
```

실행하면 팔로워암의 토크가 풀리고, 손으로 팔을 움직이면 화면 속
로봇이 30Hz로 따라옵니다.

⚠ 토크가 풀리면 팔이 중력에 처집니다. 실행 전에 팔을 잡거나 눕혀
두세요.

### 3. 시뮬 → 실기 (실기 구동)

```bash
python -m src.mujoco_sim.sim_to_robot --port /dev/ttyACM0 --id my_follower
```

뷰어의 Control 슬라이더를 움직이면 실기 팔이 따라 움직입니다.
시작 시 시뮬 자세를 실기 현재 자세에 맞추므로 팔이 튀지 않습니다.

⚠ 실기가 움직이는 실습입니다.
- 팔 주변 공간을 확보하고 시작하세요.
- 관절 이동량은 전송 1회당 `--max_step_deg`(기본 3도)로 제한됩니다.
  첫 실행은 `--max_step_deg 1`로 낮춰 방향부터 확인하길 권합니다.
- 비상 시 뷰어를 닫거나 Ctrl+C — 전송이 즉시 멈춥니다.

## 인자

| 인자 | 기본값 | 설명 |
|------|--------|------|
| `--port` | (필수) | 팔로워암 시리얼 포트 |
| `--id` | (필수) | 캘리브레이션에 쓴 로봇 id |
| `--scene` | 동봉 `scene.xml` | 다른 MJCF scene을 쓸 때 지정 |
| `--max_step_deg` | `3.0` | (sim_to_robot 전용) 전송 1회당 관절 이동량 상한(deg) |

## 파일 구성

| 파일 | 역할 |
|------|------|
| `src/mujoco_sim/so101/` | SO-101 MJCF 모델·메시 (SO-ARM100 저장소에서 동봉, Apache-2.0) |
| `src/mujoco_sim/sim_bridge.py` | 실기(deg·0~100) ↔ 시뮬(rad) 단위 변환, 관절 매핑 |
| `src/mujoco_sim/digital_twin.py` | 실기 → 시뮬 미러링 |
| `src/mujoco_sim/sim_to_robot.py` | 시뮬 → 실기 구동 (이동량 제한 포함) |
| `requirements.txt` | 의존성 (lerobot 0.6.0 · mujoco 3.12.0) |

## 자주 겪는 문제

- **`ModuleNotFoundError: No module named 'src'`** — 레포 루트가 아닌
  곳에서 실행한 경우입니다. `cd so101_mujoco_edu` 후 다시 실행합니다.
- **포트 권한 오류** — `sudo usermod -aG dialout $USER` 후 재로그인하거나
  `sudo chmod 666 /dev/ttyACM0`으로 임시 허용합니다.
- **캘리브레이션 파일 없음** — `--id`가 캘리브레이션 때 쓴 값과 다릅니다.
  `lerobot-calibrate`를 같은 id로 다시 실행합니다.
