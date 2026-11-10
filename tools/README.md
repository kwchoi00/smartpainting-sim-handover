# tools — 계측·분석 도구

리포의 본 코드가 아니다. **시뮬레이션을 돌려서 숫자를 뽑을 때** 쓰는 일회성
도구들이고, README·docs/ 의 실측값(덱 출렁임 각도, 위치 오차 mm, 스케일 k 등)이
전부 여기서 나왔다. 빌드에 안 들어가므로 `python3 tools/xxx.py` 로 직접 부른다.

ROS 쪽 도구는 시뮬이 떠 있어야 하고, 먼저 `source install/setup.bash` 를 할 것.

## 1. 기록 — 돌려 놓고 CSV 를 받는다

| 파일 | 무엇을 받나 |
|---|---|
| `rec2.py` | **주로 이걸 쓴다.** 참값 / ESKF / 오도메트리 + IMU + 관절 + patrol 단계로그를 20 Hz 로 한 CSV 에 |
| `recorder.py` | 덱 출렁임 전용. 참값 자세 + 관절 + 단계로그. 덱의 **세계좌표 yaw** = 바퀴부 yaw + `body_turn` 관절각 |
| `gzstate.py` | `gz topic -e` 를 한 번 찍어 관절값·모델 포즈를 뽑는다 (ROS 없이) |
| `rec.py` | gz `/joint_states` 텍스트 덤프를 CSV 로 파싱 |
| `stream.py` | 거대한 `gz topic -e` 덤프를 스트리밍으로 파싱 (N 개마다 하나) |

```bash
python3 tools/rec2.py /tmp/run.csv 600     # 600 초 기록
```

⚠ `gz topic -e` 를 파일로 그냥 받으면 안 된다. `/joint_states` 가 1000 Hz 라
  몇 분 만에 수백 MB 가 된다. 그래서 `rec.py`/`stream.py` 가 있고, 결국은
  ROS 쪽에서 20 Hz 로 받는 `rec2.py` 로 옮겼다.

## 2. 주행 — 재현 가능한 최소 동작만 시킨다

| 파일 | 무엇을 시키나 | 왜 |
|---|---|---|
| `drive_cycles.py` | 앞뒤 왕복만 | 스케일 `k` 는 **가감속이 있어야** 관측된다. 등속·정지에서는 안 보인다 |
| `jack_cycles.py` | 잭업 → 바퀴부 90° 회전 반복 (주행 없음) | 필터가 가장 취약한 구간만 떼어 본다 |

```bash
python3 tools/drive_cycles.py 3 1.0 0.2   # 3 왕복, 1 m, 0.2 m/s
python3 tools/jack_cycles.py 3            # 잭업 회전 3 회
```

풀코스 한 바퀴는 이게 아니라 `ros2 run sp_gazebo patrol.py` 다.

## 3. 분석 — CSV 를 읽어 숫자를 낸다

| 파일 | 무엇을 내나 |
|---|---|
| `analyze.py` | patrol 단계로그(`▶`)로 구간을 잘라 **구간별 덱 yaw 흔들림** |
| `an1.py` | 시작점 기준 국소좌표로 옮겨 참값·ESKF 궤적을 나란히 |
| `an2.py` | `[t0,t1]` 구간 표 — z, 참yaw, ESKFyaw, 오차, 오도메트리 |
| `an3.py` | `[t0,t1]` 구간 원시값 덤프 — 가속도·각속도까지 |

```bash
python3 tools/analyze.py /tmp/run.csv
python3 tools/an2.py /tmp/run.csv 120 180
```

---

⚠ 이 세션에서 소스를 고칠 때 쓴 **패치 스크립트들은 안 넣었다.** 이미 다 적용됐고,
  다시 돌리면 지금 소스를 망가뜨린다. 원본 CSV·로그·PNG 도 안 넣었다 (약 11 MB,
  다시 돌리면 나온다). 거기서 얻은 **결론과 숫자는 README 와 docs/ 에**
  남아 있다.
