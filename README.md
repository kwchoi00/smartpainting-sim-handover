# smartpainting-sim-handover

선박 블록 내부 Grit 분사 크롤러 로봇의 **IMU + 휠 오도메트리 단독 위치·자세 추정** 검증용
ROS2 / Gazebo 시뮬레이션 — **인수인계용 정리본**.

- **ROS2 Jazzy** + **Gazebo Harmonic** (gz-sim 8), Ubuntu 24.04 (VMware VM 에서 개발)
- 원본 개발 리포(`kwchoi00/smartpainting-sim`, 비공개)의 **2026-08-19 시점**(`53e744e`) 내용이다.
  그 뒤에 학회 발표·논문 준비용으로 추가한 실험·그림·플랫폼은 넣지 않았다.
- 연구 노트·설계 기록은 연구실 Obsidian 볼트 `Research/스마트페인팅/` 에 있다.

## 왜 이 시뮬레이션인가

Grit 모사장치 실험에서 **고농도 분진 시 LiDAR·Radar 모두 사용 불가**임이 확인됐다
(Grit가 금속이라 전파도 강하게 반사). 외수용 센서가 전부 무력한 구간에서
IMU + 휠 오도메트리만으로 얼마나 버티는지를 정량화하는 것이 목표.

**목표는 "정확한 위치추정"이 아니라 "허용 오차를 넘기까지의 시간·거리 한계"다.**
절대 위치 관측이 없으므로 드리프트는 유계가 아니며, 이는 튜닝으로 없앨 수 없다.

## 인수인계 요약 — 지금 어디까지 와 있나

| 영역 | 상태 | 자세히 |
| --- | --- | --- |
| 블록 월드 | 실 도면(`STANDARD BLOCK ASSY.stp`, 2026-08-14 재출도)을 실척으로 변환해 사용 중 | [docs/world.md](docs/world.md) |
| 로봇 모델 | 6안 도면(`모바일 로봇 ASSY.stp`) 기반 — 바퀴부 / 몸체부 / 전후단부 / FANUC CRX-20iA/L 6축 팔 | [docs/robot.md](docs/robot.md) |
| 동작 | `maneuver.py`(주행→회전→착지→구획 왕복), `patrol.py`(ㅁ자 순회) | [docs/robot.md](docs/robot.md) |
| 자기위치 추정 | 9상태 ESKF `[x, y, ψ, vx, vy, bax, bay, bg, k]` + NHC + ZUPT + slip 주입 + 평가 | [docs/localization.md](docs/localization.md) |
| 핵심 결과 | 1 m 왕복 3회(6.1 m), slip +3.98 % → k 수렴 **+4.05 %**, ESKF 최대 오차 **23 mm** (휠 단독 39 mm) | [결과](docs/localization.md#결과--스케일-오차는-가감속이-있으면-잡힌다) |
| 계측 도구 | 기록·주행·분석 일회성 스크립트 | [tools/README.md](tools/README.md) |

**처음 보는 사람은** 아래 [설치](#설치) → [실행 — 터미널 4개](#실행--터미널-4개) 순서로 한 번
돌려 본 뒤, [docs/localization.md](docs/localization.md) 의 결과 절을 읽으면 된다.

## 문서 구성

| 문서 | 내용 |
| --- | --- |
| `README.md` (이 파일) | 개요, 설치, 실행, 토픽, 패키지, 남은 일 |
| [docs/robot.md](docs/robot.md) | 동작 시나리오, 덱 출렁임 원인(블록 거더), ㅁ자 순회, 전후단부·시저 리프트, 매니퓰레이터 |
| [docs/localization.md](docs/localization.md) | 센서 배치, 토픽 체인, ESKF 구조, 겪은 함정들, slip 주입, monitor / lap_eval, 결과 |
| [docs/world.md](docs/world.md) | STEP 도면 → Gazebo 변환, 블록 치수, 충돌 메시 확인, 파라메트릭 월드 |
| [docs/vmware.md](docs/vmware.md) | VMware 에서 GUI 가 깨질 때 |
| [tools/README.md](tools/README.md) | 계측·분석 도구 |

## 설치

```bash
sudo apt install ros-jazzy-desktop ros-jazzy-ros-gz \
                 ros-jazzy-robot-localization ros-jazzy-teleop-twist-keyboard \
                 python3-yaml

mkdir -p ~/ws && cd ~/ws
git clone https://github.com/kwchoi00/smartpainting-sim-handover.git
cd smartpainting-sim-handover
colcon build --symlink-install
source install/setup.bash
```

⚠ **블록·로봇 메시(STL)는 리포에 없다** (수십 MB). 빌드 전에 원본 STEP 도면에서 뽑아야
로봇과 도면 월드가 보인다. 원본 STEP 은 리포 밖에서 관리하므로 전임자에게 따로 받을 것.

```bash
# 로봇 (모바일 로봇 ASSY.stp → src/sp_description/meshes/*.stl). STEP 파싱에 13분쯤 걸린다
python3 src/sp_gazebo/scripts/step_to_robot.py "모바일 로봇 ASSY.stp" -o src/sp_description
# 블록 (STANDARD BLOCK ASSY.stp → src/sp_gazebo/models/ship_block_cad/meshes/)
python3 src/sp_gazebo/scripts/step_to_gazebo.py "STANDARD BLOCK ASSY.stp" \
        -o src/sp_gazebo --scale 0.001 --up-axis y --visual-size 0.30
```

자세한 옵션은 [docs/world.md](docs/world.md), [docs/robot.md](docs/robot.md) 참조.
파라메트릭 월드(`ship_block_mockup` / `ship_block_full` / `ship_block_drift`)는 블록 메시 없이 뜬다.

## 실행 — 터미널 4개

**모든 터미널에서 먼저 source 한다.**

```bash
cd ~/ws/smartpainting-sim-handover
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

**최소 구성은 1 → 2 → 4 다.** 3번(실시간 창)은 있으면 좋고, 없어도 터미널 2가
2 초마다 오차를 찍어 준다. 순서는 지킬 것 — Gazebo 가 떠 있어야 나머지가 붙는다.

### 터미널 1 — Gazebo

```bash
ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world \
    x:=-2.68 y:=-0.8775 z:=0.53 rviz:=false software_render:=true
```

| 인자 | |
| --- | --- |
| `software_render:=true` | VMware 에서 화면이 깨지면. 대신 느리다 (실시간의 30 % 안팎) |
| `gui:=false` | 헤드리스. 측정만 할 때 빠르다 |
| `robot:=false` | 월드 형상만 본다 (기본값은 `true`) |
| `world:=ship_block_drift` | 론지만 40 m 깐 월드 — 긴 직진 드리프트용 (`x:=-18 y:=0 z:=0.46`) |
| `world:=ship_block_full` / `ship_block_mockup` | 파라메트릭 월드 (실블록 스케일 / 축소 모형) |

⚠ **`y` 는 아무 값이나 쓸 수 없다.** 바퀴부 양쪽 바퀴가 각각 이웃한 론지에
걸터앉아야 하므로 **이웃 론지 두 개의 중점**이어야 한다 —
`-0.0325 / 0.8125 / 1.6575 / -0.8775 / -1.7225`.
`x` 는 **뒤쪽(−X) 횡프레임에 최대한 붙인 −2.68** 이다 (실측: 후진시키면 −2.697 에서
횡프레임에 닿는다). 여기서 시작하면 앞으로 **1.45 m** 를 쭉 갈 수 있다 —
횡프레임 사이가 5.91 m 인데 로봇이 4.43 m 라 총 주행 여유가 1.48 m 뿐이고,
가운데서 시작하면 그게 앞뒤로 쪼개진다.
`z` 는 론지 플랜지 상면(0.520) + 여유.

⚠ 벽에 붙여 시작하므로 **`maneuver.py` 는 `-d 0.5`(앞으로)로 줘야 한다.**
기본값 `-d -0.5` 는 뒤로 가라는 뜻인데 뒤가 막혀 있다.

### 터미널 2 — ESKF (엔코더 → slip 주입 → 필터 → 평가)

```bash
ros2 launch sp_localization eskf.launch.py
ros2 launch sp_localization eskf.launch.py csv:=/tmp/run1.csv                  # 궤적 기록
ros2 launch sp_localization eskf.launch.py slip:=false odom_topic:=/odom_enc   # slip 없이
```

노드 4개가 같이 뜬다: `wheel_odom` → `slip_injector` → `eskf` → `eval`.
2 초마다 `주행 x.xx m | 오도메트리 xxx mm | ESKF xxx mm` 가 찍힌다.

> `Package 'sp_localization' not found` 가 가끔 난다. 그땐 한 줄 더:
> `source install/sp_localization/share/sp_localization/local_setup.bash`

### 터미널 3 — 실시간 비교 창

```bash
ros2 run sp_localization monitor
ros2 run sp_localization monitor --ros-args -p save:=/tmp/lap.png   # PNG 로 남긴다
```

0.2 초마다 갱신되는 창 하나: ① XY 궤적 ② **위치 오차 누적 RMSE** ③ |방위각 오차|.
**한 바퀴를 돌면 자동으로 멈추고** 최종 수치를 창 위에 찍는다.

`monitor` 는 `lap_eval` 과 **같은 프로그램**이고 한 바퀴 판정 거리 기본값만 다르다
(`away` 0.30 / `close` 0.15). 자세한 것은 아래
[실시간 비교 — `monitor`](docs/localization.md#실시간-비교--monitor) 참조.

### 터미널 4 — 로봇 동작

```bash
ros2 run sp_gazebo maneuver.py -d 0.5      # 주행→회전→착지→구획 왕복 (전 과정)
                                          # ⚠ 스폰이 −X 벽에 붙어 있어 +방향으로 줘야 한다
ros2 run sp_gazebo maneuver.py --no-sweep # 착지까지만
ros2 run sp_gazebo maneuver.py --cycle    # 위 + 주행 자세 복귀
ros2 run sp_gazebo maneuver.py --home     # 복귀만
ros2 run sp_gazebo maneuver.py -d 1.0 -s 0.15    # 회전 전 주행거리 / 속도
```

수동 조종만 하려면:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 끝나고 — 그래프

```bash
ros2 run sp_localization plot /tmp/run1.csv
```

4칸 PNG 가 CSV 옆에 저장된다: XY 궤적 / 오차−시간 / **오차−거리** / 방위각 오차.
세 번째 칸이 직선이면 스케일 오차라는 뜻이다.

### ⚠ 프로세스 정리 — 반드시 확인할 것

**터미널을 닫아도 gz 가 살아남는다.** 남은 시뮬이 같은 토픽에 계속 publish 해서
다음 측정을 통째로 오염시킨다 (실제로 80분 된 시뮬이 새 측정에 섞여 한 번 버렸다).

**스크립트 하나로 끝낸다:**

```bash
ros2 run sp_gazebo cleanup.sh          # 빌드 전이면 src/sp_gazebo/scripts/cleanup.sh
```

프로세스를 죽이고 `ros2 daemon stop` 까지 한 뒤 0 인지 확인해 준다.
수동으로 하려면 — `pkill -f "gz sim"` 은 **자기 자신을 먼저 잡아서 실패한다.**
PID 를 파일에 모은 뒤 죽인다:

```bash
ps -eo pid,args | grep -E "[g]z sim|[p]arameter_bridge|[r]obot_state_pub|[s]p_localization" \
  | awk '{print $1}' > /tmp/k.txt && xargs -r kill -9 < /tmp/k.txt
ros2 daemon stop                                       # ← 이것도 같이
ps -eo args | grep -cE '[g]z sim|[p]arameter_bridge'   # 0 이어야 정상
ros2 topic list                                        # /parameter_events, /rosout 만 남아야 정상
```

**⚠ `ros2 daemon stop` 을 빼먹지 말 것.** 프로세스를 다 죽여도 데몬이 낡은 토픽
목록을 들고 있어서 `ros2 topic list` 가 **유령 토픽**을 보여준다. 그걸 보고
"아직 떠 있나" 싶어 한참 헤매게 된다.

**⚠ 특히 `parameter_bridge` 가 잘 남는다.** `Ctrl+C` 로 런치를 끊어도 살아남아서
같은 토픽에 계속 붙어 있는다 (실측: 8시간 넘은 브리지 5개가 남아 있었다).

**측정 전에는 항상 한 줄로 확인한다:**

```bash
ps -eo comm | grep -c ruby        # 0 (시뮬 끄고) 또는 1 (하나만 켜고) 이어야 한다
```

이걸 안 지켜서 이 저장소에서 실제로 두 번 틀렸다:

| 증상 | 실제 |
| --- | --- |
| 실시간 계수 0.30 으로 측정 | 인스턴스 4개가 섞인 값. 단일 인스턴스로는 **0.15** |
| `patrol.py` 가 엉뚱한 자리에서 시작 | 스폰 y −0.8775 인데 −0.276 으로 읽힘 — 다른 인스턴스의 토픽 |

### GUI 카메라

**GUI 카메라는 블록 안쪽에서 시작한다.** gz 기본 시점(`-6 0 6`)은 블록 바깥 위인데
선체 블록은 사방이 막힌 상자라 회색 덩어리 하나만 보인다. `config/gui_cad_world.config`
가 카메라를 첫 구획 안(`-4.95 -2.70 2.50`)에 두어 론지 배열·횡프레임이 바로 보이게 한다.
다른 시점을 쓰려면 그 파일을 복사해 `gui_config:=<경로>` 로 넘기거나, 실행 중에 옮긴다:

```bash
gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera \
    --reptype gz.msgs.Boolean --timeout 3000 \
    --req 'pose: {position: {x: -4.95, y: -2.7, z: 2.5},
                  orientation: {x: -0.0887, y: 0.2116, z: 0.3763, w: 0.8977}}'
```

RViz 를 켜면(`rviz:=true`) **주황 화살표(휠 오도메트리)** 와
**초록 화살표(참값)** 가 벌어지는 정도가 곧 드리프트다.

## 토픽

아래는 전부 **로봇이 스폰됐을 때**(`robot:=true`, 기본값) 나오는 토픽이다.
`robot:=false` 면 `/clock` 외에는 뜨지 않는다.

| 토픽 | 타입 | 용도 |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 조종 입력 |
| `/joint_states` | `sensor_msgs/JointState` | 전 관절. 구동륜 26개가 **엔코더 원시값** |
| `/imu` | `sensor_msgs/Imu` | IMU 200 Hz — **추정 입력** |
| `/odom` | `nav_msgs/Odometry` | gz DiffDrive 가 내주는 값 — **추정에 쓰지 않는다** |
| `/ground_truth` | `nav_msgs/Odometry` | 참값 — **평가 전용, 알고리즘에 넣지 말 것** |

관절 지령은 `/body_lift` `/body_turn` `/front_lift` `/rear_lift` `/front_turn`
`/rear_turn` `/rail_cmd` (전부 `std_msgs/Float64`).
`/arm_j1` … `/arm_j6` (`std_msgs/Float64`)는 팔 6축 위치 [rad] 다.
`/scissor/traj` (`trajectory_msgs/JointTrajectory`)는 `scissor_driver` 가 내는
**시저 링키지 종동 지령**이다 — 사람이 쏠 일은 없다 ([두 곳의 시저](docs/robot.md#두-곳의-시저) 참조).
추정 쪽 토픽(`/odom_enc` `/odom_slip` `/odom_eskf`)은 [docs/localization.md](docs/localization.md#토픽-체인) 참조.


## 패키지

```
src/
├── sp_description/     로봇 URDF/xacro (6안 도면 로봇)
│   ├── urdf/params.xacro          치수·IMU 노이즈 등 파라미터 (전부 여기 모여 있다)
│   ├── urdf/crawler.urdf.xacro    링크·관절 구조
│   ├── urdf/crawler.gazebo.xacro  gz 플러그인 (DiffDrive, IMU, 관절 제어)
│   └── meshes/                    ← step_to_robot.py 가 채운다 (git 제외)
├── sp_gazebo/          월드·런치·브리지·동작 스크립트
│   ├── config/block_*.yaml         파라메트릭 블록 형상
│   ├── config/gui_cad_world.config GUI 카메라(블록 안쪽) 설정
│   ├── models/ship_block_cad/      도면 블록 모델 (meshes/ 는 git 제외)
│   ├── worlds/                     cad_world / drift / full / mockup
│   ├── scripts/step_to_gazebo.py   블록 STEP → 정적 Gazebo 모델
│   ├── scripts/step_to_robot.py    로봇 STEP → 링크별 STL
│   ├── scripts/inspect_block.py    데크·주행영역 분석
│   ├── scripts/gen_world.py        YAML → SDF 생성기
│   ├── scripts/maneuver.py         주행→회전→착지 동작 시나리오
│   ├── scripts/patrol.py           ㅁ자 순회 주행 (종주행 + 횡주행)
│   ├── scripts/scissor_driver.py   시저 X자 링키지 종동 구동
│   ├── scripts/cleanup.sh          남은 시뮬 프로세스 + ros2 데몬 정리
│   └── launch/sim.launch.py
└── sp_localization/    자기위치 추정 (Python)
    ├── wheel_odom.py      관절각 → 휠 오도메트리 (/odom_enc)
    ├── slip_injector.py   slip 주입 (/odom_slip)
    ├── eskf_node.py       IMU + 엔코더 ESKF (/odom_eskf)
    ├── eval_node.py       참값 대비 오차 기록 (CSV)
    ├── lap_eval.py        한 바퀴 평가 / live_monitor.py 실시간 창
    ├── plot_paths.py      CSV → 4칸 그래프
    └── launch/eskf.launch.py
tools/                  계측·분석 일회성 스크립트 (빌드에 안 들어간다)
docs/                   주제별 상세 기록
```

## ⚠ 아직 가정치인 것

**블록 치수는 전부 실 도면 실측이다.** 남은 가정치는 로봇·센서 쪽이다.

| 항목 | 왜 중요한가 |
| --- | --- |
| IMU 노이즈 파라미터 | **가장 중요.** 지금 값(`params.xacro` 의 `gyro_*` / `accel_*`)은 소비자급 MEMS 의 대략치라 낙관적일 수 있다. 실물 Livox Mid-360 정지 로깅 → Allan variance 로 교체할 것 (대응 관계는 `params.xacro` 주석에 있다) |
| IMU 장착 위치 | 도면에 센서가 없다. 지금은 바퀴부 중심(레버암 0)의 이상적 배치 — [센서 배치](docs/localization.md#센서-배치-이상적-배치-기준) |
| slip 크기 | `slip_injector` 의 반경오차·상시 slip·돌발 slip 은 가정치다. 실측 근거가 없다 |
| 질량·관성 | 도면에 재질 정보가 없어 추정. 가감속 응답·미끄러짐에 영향 |
| **전후단부 상하 스트로크** | 도면 450 mm 로는 **블록 거더(0.590/0.660)를 못 넘어 회전이 막힌다.** 시뮬은 640 mm 로 열어 뒀다 — 190 mm 부족분을 시저로 낼 수 있는지 확인할 것 ([해당 절](docs/robot.md#전후단부를-돌릴-때-덱이-출렁인다--블록-거더에-걸리는-것이었다)) |

## 알려진 한계

- **드리프트는 유계가 아니다.** k(엔코더 스케일)가 가감속·정지 구간에서 잡히긴 하지만,
  절대 위치 관측이 없으면 오차는 계속 쌓인다. 유계로 만들려면 론지 이음매 감지,
  LiDAR 정합 같은 절대 관측이 하나는 있어야 한다.
- **등속 편도 주행에서는 k 가 관측되지 않는다** — 융합 이득이 거의 없다
  ([결과](docs/localization.md#결과--스케일-오차는-가감속이-있으면-잡힌다)).
- **ESKF 는 2D 다.** 차체 롤·피치를 추정하지 않으므로, 차체가 기울면 중력이 수평 가속으로
  새어 들어간다. 론지 평행 주행(기울기 ≈ 0°)에서만 검증했다.
- **slip 은 물리가 아니라 주입이다.** `DiffDrive` 는 미끄러짐 없는 구동이라,
  `slip_injector` 가 엔코더 오도메트리에 인위적으로 얹는다.
- 6안 모델은 구동륜이 촘촘해 느리다 — 실시간 계수 0.15~0.19
  ([성능](docs/robot.md#-성능--구동륜-촘촘-배치의-대가)).
- `eskf_node.py` 주석이 말하는 `imu_odom.txt`(ESKF 유도 정리)는 리포에 없다 — 연구 노트 쪽에 있다.

## 도면 이력

| 무엇 | 상태 |
| --- | --- |
| 블록 `240305_STANDARD BLOCK ASSY.stp` (구) | 사용 안 함 — 신 도면과의 차이는 [docs/world.md](docs/world.md#구-도면240305_standard-block-assystp-과의-차이) |
| 블록 `STANDARD BLOCK ASSY.stp` (2026-08-14 재출도) | **현재 사용** |
| 로봇 — 론지 걸터타기 크롤러 (초기 가설 모델) | 제거됨. 도면이 아니라 가설이었다 |
| 로봇 `모바일 로봇 ASSY.stp` (6안, 2026-08-14) | **현재 사용** (`sp_description`) |

구 모델·구 도면 시절 코드는 원본 리포(비공개) 이력에만 있다.
구 블록 도면은 `9a06491` 직전, 걸터타기 크롤러는 `e909117` 직전 커밋에서 볼 수 있다.

## 남은 일

- [ ] 실물 Livox 정지 로깅 → Allan variance → IMU 노이즈 파라미터 교체
- [ ] 실제 IMU 장착 위치 확정 → 레버암·회전 보정 반영 (상판에 실으면 몸체부와 같이 돈다)
- [ ] slip 크기 실측 → `slip_injector` 파라미터 교체
- [ ] 전후단부 스트로크 190 mm 부족분 — 설계 쪽 확인
- [ ] 차체가 기우는 주행(론지 횡단 등)을 하려면 ESKF 에 롤·피치 추정 + 중력 제거 필요
- [ ] 드리프트 vs 주행거리·시간 곡선, ZUPT/NHC on-off ablation
- [ ] 절대 위치 관측 후보(론지 이음매 감지 등) 검토
