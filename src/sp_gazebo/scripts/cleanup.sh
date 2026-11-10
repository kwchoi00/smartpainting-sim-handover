#!/usr/bin/env bash
# 남은 시뮬 프로세스를 전부 정리한다.
#
#     ros2 run sp_gazebo cleanup.sh
#     src/sp_gazebo/scripts/cleanup.sh        (빌드 전에도 이렇게 쓸 수 있다)
#
# ⚠ 터미널을 닫거나 Ctrl+C 로 끊어도 gz 서버와 parameter_bridge 가 살아남는다.
#   남은 것이 같은 토픽에 계속 publish 해서 다음 측정을 통째로 오염시킨다
#   (실측: 8시간 넘은 브리지 5개가 남아 있었고, 그 탓에 실시간 계수를 0.15 대신
#    0.30 으로 잘못 쟀다).
#
# ⚠ **자기 자신을 죽이지 않게 조상 PID 를 전부 뺀다.** `pkill -f` 도, grep 의
#   `[g]z` 요령도 이걸 못 막는다 — 스크립트를 부른 셸의 **명령줄에 패턴 문자열이
#   들어 있으면** 그 셸까지 잡힌다 (실제로 처음에 자기를 죽이고 멈췄다).
set -u

PAT='[g]z sim|[p]arameter_bridge|[r]obot_state_pub|[r]viz2|[s]p_localization/lib|[s]p_gazebo/lib|[r]os2 launch sp_'

# 나 자신부터 init 까지 조상 PID 를 모아 제외 목록으로 쓴다
SKIP=" $$ "
pid=$$
while [ "$pid" -gt 1 ]; do
    pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$pid" ] && break
    SKIP="$SKIP$pid "
done

VICTIMS=()
while read -r p _; do
    case "$SKIP" in *" $p "*) continue ;; esac
    VICTIMS+=("$p")
done < <(ps -eo pid,args | grep -E "$PAT")

if [ "${#VICTIMS[@]}" -gt 0 ]; then
    echo "정리할 프로세스 ${#VICTIMS[@]}개: ${VICTIMS[*]}"
    kill -9 "${VICTIMS[@]}" 2>/dev/null
    sleep 2
else
    echo "정리할 프로세스 없음"
fi

# ⚠ 데몬까지 내려야 한다. 프로세스를 다 죽여도 데몬이 낡은 토픽 목록을 들고 있어서
#   `ros2 topic list` 가 유령 토픽을 보여준다.
if command -v ros2 >/dev/null 2>&1; then
    ros2 daemon stop >/dev/null 2>&1 && echo "ros2 데몬 정지"
fi

LEFT=0
while read -r p _; do
    case "$SKIP" in *" $p "*) continue ;; esac
    LEFT=$((LEFT + 1))
done < <(ps -eo pid,args | grep -E "$PAT")
echo "남은 프로세스: ${LEFT}  (0 이어야 정상)"
[ "$LEFT" -eq 0 ] || { echo "⚠ 아직 남았다:"; ps -eo pid,args | grep -E "$PAT"; exit 1; }
