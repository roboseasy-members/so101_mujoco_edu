"""SO-102 URDF로부터 MuJoCo 모델(so102.xml)을 생성한다.

URDF의 0 rad는 CAD 자세(팔이 접힌 휴식 자세)라서 lerobot의 0°
(캘리브레이션 때의 '가동범위 중앙' 자세)와 다릅니다. 이 스크립트는
SO-101 모델(so101_new_calib)과 같은 규칙이 되도록 관절마다

1. 0점 이동: URDF 각도 ZERO_DEG 자세를 새 0으로 삼는다
   (자식 바디 프레임을 관절축 기준으로 미리 회전)
2. 회전 방향: SIGN이 -1이면 관절축을 뒤집는다
3. 가동범위: lerobot 기준 범위 RANGE_DEG를 쓴다

를 적용합니다. 결과적으로 MuJoCo 슬라이더 값(rad) = 실기 lerobot 각도(deg)를
라디안으로 바꾼 값이 됩니다.

ZERO_DEG·SIGN은 SO-101 모델의 0 자세(위팔 수직·아래팔 수평·손목 일자)와
링크 방향이 일치하도록 수치 탐색으로 구했습니다. RANGE_DEG는 SO-101과
같은 부품을 쓰는 관절은 SO-101 모델 값, 새 손목 부품(wrist_flex,
wrist_yaw)은 URDF 한계를 같은 규칙으로 옮긴 값입니다.

Example:
	python tools/build_so102_mjcf.py
"""

import argparse
import math
import shutil
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Final

import mujoco
import numpy as np

REPO_ROOT: Final[Path] = Path(__file__).resolve().parents[1]
DEFAULT_DESC: Final[Path] = (
	REPO_ROOT.parent / 'so-102' / 'soarm101_7dof_description')
DEFAULT_OUT: Final[Path] = REPO_ROOT / 'src' / 'mujoco_sim' / 'so102'
URDF_NAME: Final[str] = 'soarm101_7dof_relative.urdf'

# 관절: (URDF 기준 새 0점 deg, 회전 방향, lerobot 기준 범위 deg)
JOINTS: Final[dict[str, tuple[float, int, tuple[float, float]]]] = {
	'shoulder_pan': (0.0, -1, (-110.0, 110.0)),
	'shoulder_lift': (-90.0, -1, (-100.0, 100.0)),
	'elbow_flex': (90.0, -1, (-96.83, 96.83)),
	'wrist_flex': (0.0, -1, (-82.0, 66.0)),
	'wrist_yaw': (0.0, 1, (-84.0, 82.0)),
	'wrist_roll': (-90.0, -1, (-157.21, 162.79)),
	'gripper': (0.0, 1, (-10.0, 100.0)),
}
MOTOR_MESH: Final[str] = '0_sts3215'
PART_RGBA: Final[str] = '1 0.82 0.12 1'  # SO-101 모델과 같은 색
MOTOR_RGBA: Final[str] = '0.1 0.1 0.1 1'
DEFAULTS_XML: Final[str] = '''
<default>
  <default class="so102">
    <default class="visual">
      <geom type="mesh" contype="0" conaffinity="0" group="2"/>
    </default>
    <default class="collision">
      <geom type="mesh" group="3"/>
    </default>
  </default>
  <default class="sts3215">
    <joint damping="0.60" frictionloss="0.052" armature="0.028"/>
    <position kp="998.22" kv="2.731" forcerange="-3.35 3.35"/>
  </default>
</default>'''


def rotate_child_frames(spec: mujoco.MjSpec) -> None:
	"""관절마다 0점 이동·축 반전·범위를 스펙에 적용한다."""
	for joint in spec.joints:
		zero_deg, sign, (lo_deg, hi_deg) = JOINTS[joint.name]
		axis = np.array(joint.axis, dtype=float)
		turn = np.zeros(4)
		mujoco.mju_axisAngle2Quat(turn, axis, math.radians(zero_deg))
		body = joint.parent
		quat = np.zeros(4)
		mujoco.mju_mulQuat(quat, np.array(body.quat, dtype=float), turn)
		body.quat = quat
		joint.axis = axis * sign
		joint.range = [math.radians(lo_deg), math.radians(hi_deg)]


def load_spec(desc: Path, mesh_dir: Path) -> mujoco.MjSpec:
	"""URDF를 읽어 MjSpec으로 만든다 (메시 경로는 mesh_dir)."""
	text = (desc / 'urdf' / URDF_NAME).read_text().replace('../meshes/', '')
	head, tail = text.split('<link', 1)
	text = (f'{head}<mujoco><compiler meshdir="{mesh_dir}" '
		'discardvisual="false" fusestatic="false" balanceinertia="true"/>'
		f'</mujoco>\n  <link{tail}')
	return mujoco.MjSpec.from_string(text)


def style_xml(raw_xml: str) -> ET.ElementTree:
	"""raw MJCF를 SO-101 모델과 같은 구성(클래스·재질·액추에이터)으로 다듬는다."""
	root = ET.fromstring(raw_xml)
	root.set('model', 'so102')
	compiler = root.find('compiler')
	compiler.attrib = {
		'angle': 'radian', 'meshdir': 'assets', 'autolimits': 'true'}
	root.insert(1, ET.fromstring(DEFAULTS_XML))

	asset = root.find('asset')
	asset.append(ET.Element('material', name='part', rgba=PART_RGBA))
	asset.append(ET.Element('material', name='motor', rgba=MOTOR_RGBA))

	for body in root.iter('body'):
		if body.get('name') == 'base_link':
			body.set('childclass', 'so102')
		for geom in body.findall('geom'):
			is_visual = geom.get('group') == '1'
			for key in ('contype', 'conaffinity', 'group', 'density', 'type'):
				geom.attrib.pop(key, None)
			geom.set('class', 'visual' if is_visual else 'collision')
			if is_visual:
				is_motor = geom.get('mesh') == MOTOR_MESH
				geom.set('material', 'motor' if is_motor else 'part')
		joint = body.find('joint')
		if joint is not None:
			joint.attrib.pop('actuatorfrcrange', None)
			joint.set('class', 'sts3215')

	# base_link는 world에 고정(weld)되어 MuJoCo 부모-자식 충돌 필터가
	# 적용되지 않는다. 두 링크의 충돌 메시가 원래 겹쳐 있어 shoulder_pan이
	# 접촉력에 묶이므로 이 쌍만 명시적으로 제외한다.
	contact = ET.SubElement(root, 'contact')
	ET.SubElement(contact, 'exclude', body1='base_link', body2='shoulder_link')

	actuator = ET.SubElement(root, 'actuator')
	for name in JOINTS:
		joint = root.find(f".//joint[@name='{name}']")
		ET.SubElement(actuator, 'position', {
			'class': 'sts3215', 'name': name, 'joint': name,
			'ctrlrange': joint.get('range')})
	ET.indent(root, space='  ')
	return ET.ElementTree(root)


def main() -> None:
	"""엔트리 포인트."""
	parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
	parser.add_argument('--desc', type=Path, default=DEFAULT_DESC,
		help='so-102의 soarm101_7dof_description 폴더')
	parser.add_argument('--out', type=Path, default=DEFAULT_OUT,
		help='출력 폴더 (so102.xml, assets/)')
	args = parser.parse_args()

	assets = args.out / 'assets'
	assets.mkdir(parents=True, exist_ok=True)
	for stl in sorted((args.desc / 'meshes').glob('*.stl')):
		shutil.copy2(stl, assets / stl.name)

	spec = load_spec(args.desc, assets.resolve())
	rotate_child_frames(spec)
	spec.compile()
	tree = style_xml(spec.to_xml())
	out_xml = args.out / 'so102.xml'
	header = ('<!-- tools/build_so102_mjcf.py 가 so-102 URDF로부터 생성. '
		'직접 수정하지 말고 스크립트를 고쳐 다시 생성하세요. -->\n')
	out_xml.write_text(header + ET.tostring(
		tree.getroot(), encoding='unicode') + '\n')
	print(f'생성: {out_xml} (관절 {len(spec.joints)}개)')


if __name__ == '__main__':
	main()
