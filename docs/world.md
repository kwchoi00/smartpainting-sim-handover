# 월드 만들기 — 블록 도면과 파라메트릭 월드


### A. 실제 도면 (STEP) 변환 — `step_to_gazebo.py`

실 블록 도면 `.stp` / `.step` 을 Gazebo 모델로 바꾼다.

```bash
pip install gmsh          # venv 안에서. Ubuntu 24.04 는 시스템 pip 이 막혀 있다

# 1) 변환
python3 src/sp_gazebo/scripts/step_to_gazebo.py "cad/STANDARD BLOCK ASSY.stp" \
        -o src/sp_gazebo --scale 0.001 --up-axis y --visual-size 0.30

# 2) 어디에 스폰할지 확인 (데크 개구부 때문에 반드시 필요)
python3 src/sp_gazebo/scripts/inspect_block.py "cad/STANDARD BLOCK ASSY.stp" \
        --scale 0.001 --up-axis y --robot-width 0.63 --no-map

# 3) 실행
colcon build --packages-select sp_gazebo && source install/setup.bash
ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world x:=-2.68 y:=-0.8775 z:=0.53
```

**`inspect_block.py` 를 건너뛰지 말 것.** 선박 블록 데크에는 맨홀·경감공이 뚫려 있어
가운데로 스폰하면 로봇이 그대로 구멍으로 빠지는 도면이 흔하다.
(현 도면은 확인 결과 바닥판이 통판이었다 — 확인했기 때문에 아는 것이다.)

`inspect_block.py` 는 step_to_gazebo 와 **같은 좌표 변환**을 적용해서 출력하므로,
찍어주는 x/y/z 를 그대로 launch 인자에 넣으면 된다.

조선 CAD를 그대로 넣으면 거의 항상 세 가지가 문제가 되는데, 스크립트가 자동 처리한다:

| 문제 | 처리 |
| --- | --- |
| **단위** — STEP은 보통 mm, Gazebo는 m | 바운딩박스로 추정해 스케일. `--scale 0.001` 로 직접 지정 가능 |
| **up 축** — 조선 CAD 는 Y-up 인 경우가 많다 | `--up-axis y` 로 돌려세운다. **안 주면 블록이 옆으로 눕는다** |
| **원점** — CAD 원점이 형상에서 수십 m 밖 | `--origin floor-center` (기본): XY 중심 + 바닥면을 z=0 에 |
| **폴리곤 수** — 그대로 삼각화하면 충돌 메시가 수십만 삼각형 | visual / collision 을 다른 해상도로 생성 |

주요 옵션:

```bash
--scale 0.001            # 단위 추정이 틀렸을 때
--up-axis x|y|z          # CAD 의 '위' 축. 조선 도면은 보통 y
--visual-size 0.15       # visual 메시 최대 변 길이 [m]
--collision-size 0.40    # collision 메시. 물리 성능 직결 — visual 보다 크게
--origin center|floor-center|min|none
```

**up 축을 어떻게 아는가** — 부품 이름을 보면 된다.
`BOTTOM PLATE` 로 이름 붙은 솔리드의 **가장 얇은 축(= 판 법선)이 up 축**이다.
현 도면은 `STANDARD BLOCK_BOTTOM PLATE` 가 Y 법선이라 `--up-axis y`.
`inspect_block.py` 에도 **같은 값을 줘야 한다** — 다르면 스폰 좌표가 안 맞는다.

**충돌 메시 삼각형 수가 성능을 지배한다.** 6만 개를 넘으면 경고가 나온다.
느리면 `--collision-size` 를 키울 것 — 0.4 → 0.8 로 하면 대략 1/4로 준다.
visual 은 보기용이라 크게 잡아도 물리에 영향이 없다.

생성물:

```
src/sp_gazebo/
├── models/ship_block_cad/
│   ├── model.sdf  model.config     ← 커밋됨
│   └── meshes/visual.stl           ← STEP 에서 뽑아 커밋해 둠
│       meshes/collision.stl
└── worlds/ship_block_cad_world.sdf ← 커밋됨
```

### 현재 도면 — `STANDARD BLOCK ASSY.stp` (2026-08-14 재출도)

Autodesk Inventor 2025, AP214, **mm 단위**, 부품 13개 / 솔리드 46개.
파일명에서 `240305_` 가 빠졌지만 내부 제품명은 여전히 `240305_STANDARD BLOCK ASSY` 다.

> **⚠ 이 도면은 Y-up 이다.** `--up-axis y` 를 반드시 줄 것.
> 안 주면 블록이 옆으로 누운 채 변환되고, 그 상태에서 뽑은 형상 분석은 전부 틀린다.
> (2026-08-13 에 이 실수를 했고, 아래는 정정된 값이다.)

> **⚠ `--scale 0.001` 을 반드시 줄 것.** STEP 이 mm 라 실척으로 넣으려면 1/1000 이 필요하다.
> 자동 추정도 mm 로 맞히지만, 실척이 중요한 값이므로 명시해서 추정에 맡기지 않는다.

```bash
python3 src/sp_gazebo/scripts/step_to_gazebo.py "cad/STANDARD BLOCK ASSY.stp" \
        -o src/sp_gazebo --scale 0.001 --up-axis y --visual-size 0.30
python3 src/sp_gazebo/scripts/inspect_block.py "cad/STANDARD BLOCK ASSY.stp" \
        --scale 0.001 --up-axis y --robot-width 0.63 --no-map
ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world x:=-2.68 y:=-0.8775 z:=0.53
```

변환 결과 **10.03 × 10.00 × 5.18 m — 실척 그대로**.
visual 44,554 / collision 30,458 삼각형.

구조 (Gazebo 모델 좌표 기준):

| 요소 | 위치 |
| --- | --- |
| 바닥판 (10 mm) | **z = 0.060 ~ 0.070**. 10 × 10 m 전면 **구멍 없는 통판** |
| 상판 / 2층 바닥 (10 mm) | **z = 3.172 ~ 3.182** → 1층 층고 **3.11 m**, 2층 여유 2.00 m |
| **T-bar 450×150 론지 10개** | **바닥판 위에 수직으로 서 있다.** 높이 **0.450 m**, 폭 0.150 m, X 방향 10 m 관통 |
| 론지 플랜지 상면 | **z = 0.520** (바닥판 위 0.450) |
| 론지 간격 | pitch **845 mm** → 사이 통로 폭 **0.695 m** |
| 종방향 거더 2매 (10 mm) | Y ≈ +2.995 / **−3.060**, 전高 격벽. 큰 경감공 있음 |
| **거더 수평 보강대 상면** | **z 0.660 (우현) / 0.590 (좌현)** — 론지보다 70~140 mm 높다. 전후단부를 돌릴 때 **이 높이가 필요 스트로크를 정한다** |
| 횡프레임 2매 (90 mm) | X ≈ −4.97, **+1.03** → 사이 순간격 **5.91 m** + 바깥 3.91 m |

#### 구 도면(`240305_STANDARD BLOCK ASSY.stp`) 과의 차이

솔리드 수·부품명·치수는 전부 같다. 실제로 달라진 것은 세 가지뿐이다.

| 항목 | 구 | 신 | 영향 |
| --- | --- | --- | --- |
| **전체 z 오프셋** | 바닥판 상면 0.010 | 바닥판 상면 **0.070** | 론지 상면 0.460 → **0.520**. 스폰 z 전부 +0.060 |
| **횡프레임 작업자 맨홀** | 700 × 1000 mm | **600 × 800 mm** | 통행 판정은 그대로 (아래) |
| **좌현 GIRDER 1매 위치** | y −2.995, z 0.010~3.112 | y **−3.060**, z **0.000~3.102** | z 기준면이 바뀐 원인 |

**Y 좌표·론지 pitch·주행 통로는 하나도 안 바뀌었다.** 스폰 Y 값들은 그대로 쓴다.

⚠ **z 가 통째로 올라간 것은 도면 오류로 보인다.** 좌현 거더 하나가 우현 짝에 비해
(+26, −65, −70) mm 어긋나 있어서 **바닥 외판을 70 mm 뚫고 내려가 있다.** 실선체에서는
불가능한 형상이다. `--origin floor-center` 는 형상 전체의 최저점을 z=0 으로 잡으므로,
이 튀어나온 거더가 기준면이 되어 나머지가 전부 60 mm 떠올랐다.

변환기는 도면을 있는 그대로 옮기는 쪽이 맞으므로 보정하지 않았다. 도면이 수정돼 오면
거더가 제자리로 가면서 전체가 60 mm 내려올 것이므로, 그때 다시 변환하고
스폰 z 만 고치면 된다.

**맨홀은 작아졌지만 위치는 그대로다.** 신 도면 맨홀은 Gazebo 좌표로
z **0.920 ~ 1.720**. 바닥판 상면이 0.070 이므로 **개구 하단이 바닥에서 0.850 m 위**다.
높이 0.85 m 를 넘지 않는 로봇은 이 횡프레임을 통과할 수 없다.

**바닥에 구멍은 없다.** 1층 바닥 100 m² 전체가 통판이다.
로봇이 빠질 데크 개구부는 이 도면에 존재하지 않는다.

**주행을 제약하는 것은 론지다.** 바닥을 달리는 로봇은 론지 사이
**0.695 m 통로**를 X 방향으로만 갈 수 있다. 통로 안에서 방향 전환은 사실상 불가능하다.

```
Y →   -4.79  -3.95     -2.22 -1.38 -0.53  0.32  1.16  2.01      3.77  4.62   (론지 y0, m)
       ██     ██   |거더| ██    ██    ██    ██    ██    ██  |거더| ██    ██
         0.695       ↑ 론지 사이 통로 폭 0.695 m
```

**스폰 Y = 인접 론지 두 개의 중점**: `-4.29, -1.72, -0.88, -0.0325, 0.81, 1.66, 4.27`

거더(Y ≈ +2.995 / −3.060)가 전高 격벽이라 통로를 좌·중·우 세 구역으로 나눈다.

**횡프레임(X ≈ +1.03)은 면적 기준 대부분이 개구지만 로봇은 못 지나간다.** 개구가 전부
로봇 높이 위(작업자 맨홀)에 있다. 론지 위 로봇이 차지하는 z 0.52~0.91 대에서는
Y 전 구간이 막혀 있다 (맨홀 하단 z = 0.920).
→ **직선 주행거리는 두 횡프레임 사이 5.91 m 로 제한**된다. 통행 가능성은 면적 개구율이
아니라 **로봇 높이대의 개구**로 판단할 것.
(예전에 적어둔 5.98 m 는 프레임 중심선 기준이라 판 두께 90 mm 를 뺀 값이 맞다.)

조명은 그림자를 끄고 환경광을 높였다. 사방이 막힌 블록이라 그림자를 켜면 내부가
새까맣게 되고, 그렇다고 환경광만 두면 면 구분이 사라진다. 방향이 다른 무그림자
조명 3개로 음영만 준다.


### 충돌 메시가 올라갔는지 확인하는 법

구동 로그에 이런 줄이 뜨는데 **정상이다. 무시할 것.**

```
[Dbg] [SDFFeatures.cc:332] Mesh construction from an SDF has not been implemented
                           yet for dartsim. Use AttachMeshShapeFeature to use mesh shapes.
[Dbg] [SDFFeatures.cc:864] The geometry element of collision [collision] couldn't be created
```

gz-sim 은 충돌체를 만들 때 `ConstructSdfCollision` 을 먼저 시도하고, 메시면 여기서 실패한 뒤
`AttachMeshShapeFeature` 로 다시 붙인다. 위 두 줄은 그 1차 시도 기록이다.
레벨이 `[Err]` 가 아니라 `[Dbg]` 인 게 단서.

충돌이 실제로 살아 있는지는 로그가 아니라 **로봇 위치로 확인한다**:

```bash
ros2 topic echo /ground_truth --once
```

`z` 가 **론지 플랜지 상면 0.520** 이면 정상이다 (`/ground_truth` 는 `base_footprint` 기준).
충돌 메시가 죽었으면 블록 아래 지면(z ≈ 0)까지 떨어진다.
`/ground_truth` 는 odom 상대가 아니라 월드 좌표를 준다.

### B. 파라메트릭 월드 — `gen_world.py`

론지 개수·간격을 바꿔가며 실험할 때. 도면 없이도 쓸 수 있다.

```bash
# config/block_mockup.yaml 을 고친 뒤
python3 src/sp_gazebo/scripts/gen_world.py \
        src/sp_gazebo/config/block_mockup.yaml \
        -o src/sp_gazebo/worlds/ship_block_mockup.sdf
```

생성기는 로봇이 론지 사이·맨홀을 통과할 수 있는지 검사해 경고를 낸다.

> 도면 월드는 "실제 그대로", 파라메트릭 월드는 "변수를 바꿔가며".
> 드리프트가 통로 폭·회전 횟수에 얼마나 민감한지 보려면 B가 필요하다.

