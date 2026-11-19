"""digital_twin·sim_to_robot 루프를 가짜 로봇·가짜 뷰어로 돌려 본다."""

from pathlib import Path

import pytest

from src.mujoco_sim import digital_twin, sim_to_robot
from src.mujoco_sim.sim_bridge import ALL_JOINTS

SCENE = Path(__file__).resolve().parents[1] / 'src/mujoco_sim/so102/scene.xml'
POSE = {f'{name}.pos': 10.0 for name in ALL_JOINTS}


class FakeBus:
	def disable_torque(self) -> None:
		pass


class FakeRobot:
	"""SO102Follower 대역: 고정 자세를 돌려주고 받은 액션을 기록한다."""

	def __init__(self, config) -> None:
		self.config = config
		self.bus = FakeBus()
		self.sent: list[dict[str, float]] = []

	def connect(self) -> None:
		pass

	def disconnect(self) -> None:
		pass

	def get_observation(self) -> dict[str, float]:
		return dict(POSE)

	def send_action(self, action: dict[str, float]) -> dict[str, float]:
		self.sent.append(action)
		return action


class FakeViewer:
	"""launch_passive 대역: 5프레임 후 창이 닫힌 것처럼 동작한다."""

	def __init__(self, *args) -> None:
		self.frames = 5

	def __enter__(self):
		return self

	def __exit__(self, *args) -> None:
		pass

	def is_running(self) -> bool:
		self.frames -= 1
		return self.frames >= 0

	def sync(self) -> None:
		pass


@pytest.fixture
def robots(monkeypatch):
	created: list[FakeRobot] = []

	def make(config):
		created.append(FakeRobot(config))
		return created[-1]

	for module in (digital_twin, sim_to_robot):
		monkeypatch.setattr(module, 'SO102Follower', make)
		monkeypatch.setattr(module.mujoco.viewer, 'launch_passive', FakeViewer)
		monkeypatch.setattr(module.time, 'sleep', lambda _: None)
	return created


def test_digital_twin_loop(robots):
	digital_twin.run_twin('/dev/null', 'so102_follower', SCENE)
	assert len(robots) == 1


def test_sim_to_robot_holds_real_pose(robots):
	"""슬라이더를 건드리지 않으면 실기 현재 자세를 그대로 유지한다(튀지 않음)."""
	sim_to_robot.run_sim_to_robot('/dev/null', 'so102_follower', SCENE, 3.0)
	robot = robots[0]
	assert robot.config.max_relative_target == 3.0
	assert len(robot.sent) == 5
	for key, value in POSE.items():
		assert robot.sent[-1][key] == pytest.approx(value, abs=1.0), key
