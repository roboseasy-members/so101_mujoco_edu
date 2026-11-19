# SO-102 MuJoCo 모델

SO-102(7축) 로봇팔의 MuJoCo 모델입니다.

| 파일 | 내용 |
|------|------|
| `scene.xml` | 진입점 (바닥·조명 + `so102.xml` include) |
| `so102.xml` | 로봇 본체 MJCF — **생성 파일, 직접 수정 금지** |
| `assets/*.stl` | 메시 12개 |

## 어떻게 만들어졌나

[roboseasy/so-102](https://github.com/roboseasy/so-102)의
`soarm101_7dof_description/urdf/soarm101_7dof_relative.urdf`를
`tools/build_so102_mjcf.py`로 변환했습니다. 다시 만들려면 so-102 레포가
이 레포 옆 폴더(`../so-102`)에 있는 상태에서 레포 루트에서 실행합니다.

```bash
python tools/build_so102_mjcf.py
```

변환하면서 관절 0점과 회전 방향을 **lerobot 캘리브레이션 기준**으로
옮겼습니다. 그래서 슬라이더 값(rad)은 실기 각도(deg)를 라디안으로 바꾼
값과 같습니다. SO-101 모델(`so101_new_calib.xml`)과 같은 규칙입니다.

| 관절 | URDF 대비 0점 이동 | 회전 방향 | 범위 (deg) | 범위 출처 |
|------|------|------|------|------|
| shoulder_pan | 0° | 반전 | -110 ~ 110 | SO-101 |
| shoulder_lift | -90° | 반전 | -100 ~ 100 | SO-101 |
| elbow_flex | +90° | 반전 | -96.8 ~ 96.8 | SO-101 |
| wrist_flex | 0° | 반전 | -82 ~ 66 | SO-102 URDF |
| wrist_yaw | 0° | 그대로 | -84 ~ 82 | SO-102 URDF |
| wrist_roll | -90° | 반전 | -157.2 ~ 162.8 | SO-101 |
| gripper | 0° | 그대로 | -10 ~ 100 | SO-101 |

- 0점·방향: SO-101 모델의 0 자세(위팔 수직, 아래팔 수평, 손목 일자)와
  링크 방향이 일치하도록 수치로 구했습니다.
- wrist_yaw는 SO-101에 없는 축이라 URDF 방향을 그대로 씁니다. 실기와
  반대로 돈다면 `tools/build_so102_mjcf.py`의 `JOINTS` 표에서 부호를
  바꿔 다시 생성합니다.

## 라이선스

SO-ARM100에서 온 부품 메시(`*_so101.stl`, `0_sts3215.stl`)는 Apache-2.0
(`LICENSE` 동봉, 출처 [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100)).
SO-102 추가 부품(`7dof_*.stl`, `6_under_arm_ver3.stl`)과 URDF는 RoboSEasy 소유입니다.
