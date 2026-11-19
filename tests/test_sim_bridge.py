"""실기 없이 돌리는 SO-102 시뮬 브리지 테스트.

레포 루트에서: python -m pytest tests
"""

import math
from pathlib import Path

import mujoco
import pytest

from lerobot_robot_so102 import SO102Follower, SO102FollowerConfig
from src.mujoco_sim.sim_bridge import ALL_JOINTS, ARM_JOINTS, \
	get_gripper_range, get_qpos_index, obs_to_qpos, qpos_to_action

SCENE = Path(__file__).resolve().parents[1] / 'src/mujoco_sim/so102/scene.xml'


@pytest.fixture(scope='module')
def model() -> mujoco.MjModel:
	return mujoco.MjModel.from_xml_path(str(SCENE))


def test_model_has_seven_joints_and_actuators(model):
	"""관절·액추에이터가 7개이고 이름이 lerobot 모터 이름과 같다."""
	joints = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, i)
		for i in range(model.njnt)]
	actuators = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
		for i in range(model.nu)]
	assert tuple(joints) == ALL_JOINTS
	assert tuple(actuators) == ALL_JOINTS


def test_bridge_matches_so102_motor_bus():
	"""lerobot SO-102 팔로워 버스의 모터 순서와 브리지 관절 순서가 같다."""
	robot = SO102Follower(SO102FollowerConfig(
		port='/dev/null', id='pytest_so102'))
	assert tuple(robot.bus.motors) == ALL_JOINTS


def test_zero_pose_is_inside_every_range(model):
	"""lerobot 0°(캘리브레이션 중앙 자세)가 모든 관절 범위 안에 있다."""
	for joint_name in ARM_JOINTS:
		joint_id = mujoco.mj_name2id(
			model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
		lo, hi = model.jnt_range[joint_id]
		assert lo < 0.0 < hi, joint_name


def test_round_trip(model):
	"""obs → qpos → action 왕복 시 값이 보존된다."""
	data = mujoco.MjData(model)
	obs = {f'{name}.pos': deg for name, deg in zip(
		ARM_JOINTS, (30.0, -45.0, 60.0, -20.0, 15.0, 90.0))}
	obs['gripper.pos'] = 40.0
	obs_to_qpos(model, data, obs)
	assert data.qpos[get_qpos_index(model, 'wrist_yaw')] == pytest.approx(
		math.radians(15.0))
	action = qpos_to_action(model, data)
	assert action.keys() == obs.keys()
	for key, value in obs.items():
		assert action[key] == pytest.approx(value), key


def test_gripper_mapping_and_clamp(model):
	"""그리퍼 0~100이 모델 범위 양 끝에 대응하고, 범위 밖은 잘린다."""
	data = mujoco.MjData(model)
	lo, hi = get_gripper_range(model)
	index = get_qpos_index(model, 'gripper')
	obs = {f'{name}.pos': 0.0 for name in ARM_JOINTS}
	for percent, expected in ((0.0, lo), (100.0, hi)):
		obs['gripper.pos'] = percent
		obs_to_qpos(model, data, obs)
		assert data.qpos[index] == pytest.approx(expected)
	data.qpos[index] = hi + 0.5
	assert qpos_to_action(model, data)['gripper.pos'] == 100.0
