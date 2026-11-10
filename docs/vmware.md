# VMware 환경 주의


### GUI 가 깨지면 `software_render:=true`

```bash
ros2 launch sp_gazebo sim.launch.py world:=ship_block_cad_world \
    x:=0.5 y:=-1.75 z:=2.06 software_render:=true
```

VMware 게스트(Ubuntu 24.04, vmwgfx/SVGA3D)에서 확인한 결과 — 2026-08-13:

| 조건 | 결과 |
| --- | --- |
| Wayland + ogre2 | 화면 깨짐·깜빡임 |
| **X11** + ogre2 | 화면 깨짐·깜빡임 (세션 바꿔도 그대로) |
| X11 + ogre (v1) | **응답 없음으로 멈춤** |
| X11 + `LIBGL_ALWAYS_SOFTWARE=1` | **정상** |

즉 **렌더 엔진 문제가 아니라 그 아래 SVGA3D 드라이버 문제**다.
세션 종류(Wayland/X11)나 `render_engine` 을 바꾸는 것으로는 해결되지 않는다.
`software_render:=true` 는 `LIBGL_ALWAYS_SOFTWARE=1` 을 gz 프로세스에만 걸어 이를 우회한다.

커널은 3D 가속을 정상 보고한다(`Capabilities: ... 3D, dx, dx2, dx3`) — 가속이 꺼진 게 아니라
드라이버가 ogre 의 요구를 제대로 못 받아주는 것이다. VRAM 이 128 MB 로 작은 것도 겹친다.

**드리프트 실험 본작업에는 GUI 가 필요 없다.** `gui:=false` 헤드리스가 이 문제를 완전히
우회하고 더 빠르며 재현성도 좋다. GUI 는 형상 확인용으로만 쓰는 게 낫다.
- 그래도 느리면 헤드리스로 돌리고 RViz만 본다:
  `ros2 launch sp_gazebo sim.launch.py gui:=false`
- 드리프트 실험은 실시간일 필요가 없다. GUI 없이 배치로 돌리는 편이 빠르고 재현성도 좋다.
- STEP 파일은 VMware 공유 폴더로 넘기거나 VM 안에 직접 복사한다. 공유 폴더 경로는
  보통 `/mnt/hgfs/<공유이름>/`.

