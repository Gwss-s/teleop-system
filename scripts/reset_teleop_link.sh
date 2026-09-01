#!/bin/bash
# 遥操链路一键复位(PC 侧):热点 + PC Service 全新鲜。头显端(整机重启+保持清醒)手动。
# 用法: bash scripts/reset_teleop_link.sh   然后头显重连热点 -> App 连 10.42.0.1 -> 开 Send
# 依据: 2026-08-26 实证——AP/Service/头显连跑数日后空载 ping 退化到秒级+52%丢包,
#       全链路重启后恢复 3ms/0% (docs/teleop_usage.md §4)
set -e
echo "[reset] 重启热点..."
nmcli connection down hil-hotspot || true
sleep 2
nmcli connection up hil-hotspot
echo "[reset] 重启 PC Service..."
pkill -f "RoboticsServiceProces[s]" || true
sleep 2
setsid nohup bash /opt/apps/roboticsservice/runService.sh > /tmp/robotics_service.log 2>&1 &
sleep 4
pgrep -f "RoboticsServiceProces[s]" >/dev/null && ss -tln | grep -q 60061 \
  && echo "[reset] ✅ PC 侧就绪。剩头显:整机重启 -> 连热点 -> App 连 10.42.0.1 -> 开 Send" \
  || { echo "[reset] ❌ Service 启动失败,看 /tmp/robotics_service.log"; exit 1; }
