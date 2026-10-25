#!/usr/bin/env bash
# Jetson Orin Nano: 대회 주행 전 매번 실행하는 시스템 최적화 스크립트.
#
# 하는 일:
#   1. nvpmodel을 최대 성능 모드로 전환
#   2. jetson_clocks로 CPU/GPU/EMC 클럭을 최대로 고정 (부스트/스로틀링으로 인한
#      프레임 처리 속도 들쭉날쭉함 방지 -> 이게 바로 "렉" 체감의 큰 원인)
#   3. ROS2 DDS 디스커버리를 이 보드 안으로만 한정 (ROS_LOCALHOST_ONLY)
#      -> 같은 네트워크의 다른 장비와 불필요하게 디스커버리 패킷을 주고받지 않음
#   4. 현재 상태(전원 모드, 온도, 메모리)를 요약 출력
#
# 사용법:
#   chmod +x scripts/jetson_optimize.sh
#   ./scripts/jetson_optimize.sh
#
# 대회장 도착 후, 로봇을 켜고 주행 전에 한 번 실행해두는 걸 권장.
# sudo 비밀번호를 물어볼 수 있음.

set -e

echo "=== [1/4] 전원 모드 확인 ==="
if command -v nvpmodel >/dev/null 2>&1; then
    nvpmodel -q || true
    echo "-> 최대 성능 모드로 전환 시도 (sudo 필요)"
    # Orin Nano는 보드/JetPack 버전에 따라 MAXN 모드 번호가 다를 수 있어 0번을 우선 시도.
    # 실패해도 스크립트는 계속 진행함 (이미 최대 모드일 수도 있음).
    sudo nvpmodel -m 0 || echo "   (nvpmodel -m 0 실패 — 'nvpmodel -q --verbose'로 사용 가능한 모드 확인 후 수동 설정 권장)"
else
    echo "nvpmodel 명령을 찾을 수 없음 (JetPack이 아닌 환경?) — 건너뜀"
fi

echo
echo "=== [2/4] 클럭 고정 (jetson_clocks) ==="
if command -v jetson_clocks >/dev/null 2>&1; then
    sudo jetson_clocks
    echo "-> CPU/GPU/EMC 클럭 최대 고정 완료"
else
    echo "jetson_clocks 명령을 찾을 수 없음 — 건너뜀"
fi

echo
echo "=== [3/4] ROS2 DDS 디스커버리 범위 축소 ==="
echo "아래 두 줄을 ~/.bashrc 에 추가해두면 매번 자동 적용됩니다:"
echo "  export ROS_LOCALHOST_ONLY=1"
echo "  export ROS_DOMAIN_ID=0   # 대회장에 다른 ROS 로봇이 있다면 팀마다 다른 번호로"
export ROS_LOCALHOST_ONLY=1

echo
echo "=== [4/4] 현재 상태 ==="
if command -v tegrastats >/dev/null 2>&1; then
    echo "실시간 CPU/GPU/온도 확인: tegrastats  (Ctrl+C로 종료)"
fi
free -h | head -2
echo
echo "완료. GUI 데스크톱/브라우저 등 불필요한 프로그램은 주행 전에 꺼두는 걸 권장합니다"
echo "(특히 rqt_image_view 같은 뷰어는 디버깅 끝나면 반드시 종료)."
