#!/usr/bin/env bash
# scripts/setup_uart.sh
# Raspberry Pi UART 환경 초기 설정 스크립트
# 실행: sudo bash scripts/setup_uart.sh

set -euo pipefail

CONFIG="/boot/firmware/config.txt"   # RPi OS Bookworm 이후 경로
UDEV_RULES_SRC="$(dirname "$0")/../udev/99-esp32-uart.rules"
UDEV_RULES_DST="/etc/udev/rules.d/99-esp32-uart.rules"

# ── 루트 권한 확인 ─────────────────────────────────────────────────────────────
if [[ $EUID -ne 0 ]]; then
    echo "[오류] 이 스크립트는 sudo 로 실행해야 합니다."
    exit 1
fi

echo "=== Raspberry Pi UART 설정 시작 ==="

# ── 1. UART DT 오버레이 활성화 ────────────────────────────────────────────────
# /boot/firmware/config.txt 에 없는 오버레이만 추가
for overlay in uart2 uart3 uart4; do
    if grep -qF "dtoverlay=${overlay}" "$CONFIG"; then
        echo "[건너뜀] dtoverlay=${overlay} 이미 존재"
    else
        echo "dtoverlay=${overlay}" >> "$CONFIG"
        echo "[추가됨] dtoverlay=${overlay} → ${CONFIG}"
    fi
done

# ── 2. udev 규칙 설치 ─────────────────────────────────────────────────────────
if [[ ! -f "$UDEV_RULES_SRC" ]]; then
    echo "[오류] udev 규칙 파일을 찾을 수 없음: ${UDEV_RULES_SRC}"
    exit 1
fi

cp "$UDEV_RULES_SRC" "$UDEV_RULES_DST"
echo "[설치됨] ${UDEV_RULES_DST}"

udevadm control --reload-rules
udevadm trigger
echo "[완료] udev 규칙 재로드"

# ── 3. dialout 그룹 확인 ──────────────────────────────────────────────────────
CURRENT_USER="${SUDO_USER:-$(whoami)}"
if ! groups "$CURRENT_USER" | grep -q "dialout"; then
    usermod -aG dialout "$CURRENT_USER"
    echo "[추가됨] 사용자 '${CURRENT_USER}' → dialout 그룹"
    echo "         (적용을 위해 재로그인 또는 'newgrp dialout' 필요)"
else
    echo "[확인됨] '${CURRENT_USER}' 는 이미 dialout 그룹"
fi

# ── 4. Raspberry Pi 4 주소 확인 안내 ─────────────────────────────────────────
if grep -q "Raspberry Pi 4" /proc/device-tree/model 2>/dev/null; then
    echo ""
    echo "[주의] Raspberry Pi 4 (RP1 I/O 컨트롤러) 감지됨."
    echo "       udev/99-esp32-uart.rules 의 KERNELS 주소를 아래 명령으로 확인하세요:"
    echo "       ls -la /sys/bus/platform/devices/ | grep serial"
fi

echo ""
echo "=== 설정 완료. 재부팅 후 UART 활성화됩니다. ==="
echo "    sudo reboot"
