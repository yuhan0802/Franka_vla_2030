#!/usr/bin/env bash
set -euo pipefail
: "${ROBOT_IP:?Set ROBOT_IP to the FR3 control IP before launching}"

# Apply the host-side real-time settings that proved stable with the FR3.
# The helper enters the host namespaces from a short-lived privileged
# container; no interactive sudo password is required.  Override these for a
# different host with FCI_INTERFACE and FCI_IRQ_CPU.
FCI_INTERFACE="${FCI_INTERFACE:-enp110s0}"
FCI_IRQ_CPU="${FCI_IRQ_CPU:-2}"
docker run --rm --privileged --pid host \
  -e FCI_INTERFACE -e FCI_IRQ_CPU franka-vr:humble \
  nsenter -t 1 -m -p -- /bin/bash -lc '
    for governor in /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor; do
      echo performance > "$governor" 2>/dev/null || true
    done
    irq=$(grep -m1 "$FCI_INTERFACE" /proc/interrupts \
      | cut -d: -f1 | tr -d " ")
    if [[ -n "$irq" && -w "/proc/irq/$irq/smp_affinity_list" ]]; then
      echo "$FCI_IRQ_CPU" > "/proc/irq/$irq/smp_affinity_list"
    fi
  '

docker compose run --rm franka_vr ros2 launch franka_vr franka_twist.launch.py \
  robot_ip:="${ROBOT_IP}" use_fake_hardware:=false \
  load_gripper:="${LOAD_GRIPPER:-true}" use_rviz:="${USE_RVIZ:-true}"
