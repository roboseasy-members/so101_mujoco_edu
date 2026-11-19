"""교육자료용 SO-102 자세 이미지를 헤드리스로 렌더링한다.

관절마다 '최소 / 0 / 최대' 자세를 나란히 붙인 이미지를 만들어
교육자료 이미지 폴더에 저장합니다. 모델을 다시 생성했다면 이 스크립트도
다시 실행해 이미지를 갱신합니다.

Example:
	MUJOCO_GL=egl python tools/render_doc_images.py
"""

import argparse
import math
import os
from pathlib import Path
from typing import Final

os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image, ImageDraw, ImageFont  # noqa: E402

REPO_ROOT: Final[Path] = Path(__file__).resolve().parents[1]
SCENE: Final[Path] = REPO_ROOT / 'src' / 'mujoco_sim' / 'so102' / 'scene.xml'
SO101_SCENE: Final[Path] = (REPO_ROOT.parent / 'so101_mujoco_edu' / 'src'
	/ 'mujoco_sim' / 'so101' / 'scene.xml')
DOC_IMAGES: Final[Path] = (REPO_ROOT / '교육자료'
	/ '장 - 시뮬레이션과 디지털 트윈 (SO-102)')
FONT: Final[str] = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
WIDTH, HEIGHT = 520, 420

# 이미지 이름: (관절, 카메라(lookat, distance, azimuth, elevation))
SIDE = ((0.12, 0.0, 0.18), 0.75, 90.0, -10.0)
TOP = ((0.15, 0.0, 0.15), 0.70, 90.0, -75.0)
FRONT = ((0.25, 0.0, 0.22), 0.40, 180.0, -10.0)
GRIP = ((0.27, 0.0, 0.22), 0.30, 120.0, -25.0)
SHOTS: Final[dict[str, tuple[str, tuple]]] = {
	'so102_shoulder_pan': ('shoulder_pan', TOP),
	'so102_shoulder_lift': ('shoulder_lift', SIDE),
	'so102_elbow_flex': ('elbow_flex', SIDE),
	'so102_wrist_flex': ('wrist_flex', SIDE),
	'so102_wrist_yaw': ('wrist_yaw', TOP),
	'so102_wrist_roll': ('wrist_roll', FRONT),
	'so102_gripper': ('gripper', GRIP),
}


class Shooter:
	"""한 모델을 여러 자세로 렌더링한다."""

	def __init__(self, scene: Path) -> None:
		self.model = mujoco.MjModel.from_xml_path(str(scene))
		self.data = mujoco.MjData(self.model)
		self.renderer = mujoco.Renderer(self.model, HEIGHT, WIDTH)

	def joint_range(self, name: str) -> tuple[float, float]:
		joint_id = mujoco.mj_name2id(
			self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
		low, high = self.model.jnt_range[joint_id]
		return float(low), float(high)

	def shot(self, pose: dict[str, float], camera: tuple) -> np.ndarray:
		self.data.qpos[:] = 0.0
		for name, value in pose.items():
			joint_id = mujoco.mj_name2id(
				self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
			self.data.qpos[self.model.jnt_qposadr[joint_id]] = value
		mujoco.mj_forward(self.model, self.data)
		lookat, distance, azimuth, elevation = camera
		cam = mujoco.MjvCamera()
		cam.lookat[:] = lookat
		cam.distance, cam.azimuth, cam.elevation = (
			distance, azimuth, elevation)
		self.renderer.update_scene(self.data, cam)
		return self.renderer.render()


def label(image: np.ndarray, text: str) -> Image.Image:
	"""이미지 왼쪽 위에 글자를 얹는다."""
	canvas = Image.fromarray(image)
	draw = ImageDraw.Draw(canvas)
	font = ImageFont.truetype(FONT, 26)
	draw.rectangle((0, 0, WIDTH, 44), fill=(20, 24, 32))
	draw.text((14, 6), text, font=font, fill=(255, 255, 255))
	return canvas


def hstack(panels: list[Image.Image]) -> Image.Image:
	out = Image.new('RGB', (WIDTH * len(panels) + 8 * (len(panels) - 1),
		HEIGHT), (255, 255, 255))
	for index, panel in enumerate(panels):
		out.paste(panel, (index * (WIDTH + 8), 0))
	return out


def fmt(rad: float) -> str:
	return f'{rad:+.2f} rad ({math.degrees(rad):+.0f}°)'


def render_joints(shooter: Shooter, out_dir: Path) -> None:
	"""관절마다 최소·0·최대 자세 이미지를 저장한다."""
	for file_name, (joint, camera) in SHOTS.items():
		low, high = shooter.joint_range(joint)
		panels = [label(shooter.shot({joint: value}, camera),
			f'{joint} {fmt(value)}') for value in (low, 0.0, high)]
		hstack(panels).save(out_dir / f'{file_name}.png')
		print(f'  {file_name}.png')


def render_compare(so102: Shooter, out_dir: Path) -> None:
	"""SO-101과 SO-102의 0 자세를 나란히 놓는다."""
	if not SO101_SCENE.exists():
		print('  (so101_mujoco_edu 없음 - 비교 이미지 건너뜀)')
		return
	so101 = Shooter(SO101_SCENE)
	camera = ((0.15, 0.0, 0.17), 0.70, 120.0, -15.0)
	hstack([
		label(so101.shot({}, camera), 'SO-101 (6축) 0 자세'),
		label(so102.shot({}, camera), 'SO-102 (7축) 0 자세'),
	]).save(out_dir / 'so101_vs_so102.png')
	print('  so101_vs_so102.png')


def patch_twin_flow(out_dir: Path) -> None:
	"""디지털 트윈 흐름도의 '관절 이름 6개' 문구를 7개로 바꾼 사본을 만든다."""
	source = (SO101_SCENE.parents[3] / '교육자료'
		/ '장 - 시뮬레이션과 디지털 트윈' / '4_1_twin_flow.png')
	if not source.exists():
		return
	image = Image.open(source).convert('RGB')
	width, height = image.size
	draw = ImageDraw.Draw(image)
	scale = width / 2576
	top, bottom = int(1120 * scale), int(1200 * scale)
	draw.rectangle((0, top, width, bottom), fill=(255, 255, 255))
	font = ImageFont.truetype(FONT, int(36 * scale))
	text = '관절 이름 7개가 LeRobot 모터 이름과 MuJoCo 모델에서 같다. 이름표 불필요'
	text_width = draw.textlength(text, font=font)
	draw.text(((width - text_width) / 2, top + int(18 * scale)), text,
		font=font, fill=(107, 114, 128))
	image.save(out_dir / '4_1_twin_flow_so102.png')
	print('  4_1_twin_flow_so102.png')


def main() -> None:
	"""엔트리 포인트."""
	parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
	parser.add_argument('--out', type=Path, default=DOC_IMAGES)
	args = parser.parse_args()
	args.out.mkdir(parents=True, exist_ok=True)
	shooter = Shooter(SCENE)
	render_joints(shooter, args.out)
	render_compare(shooter, args.out)
	patch_twin_flow(args.out)


if __name__ == '__main__':
	main()
