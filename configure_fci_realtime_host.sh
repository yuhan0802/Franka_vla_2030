#!/usr/bin/env bash
# Configure the host once for the FR3 real-time control CPUs and the dedicated
# FCI NIC IRQ CPU.
# This changes GRUB and therefore requires sudo plus a reboot before it takes
# effect.  The FCI NIC IRQ itself is pinned separately by run_robot.sh.
set -euo pipefail

CONTROL_CPUS="${CONTROL_CPUS:-4,5}"
FCI_IRQ_CPU="${FCI_IRQ_CPU:-2}"
# CPU 2 is reserved for the FCI NIC interrupt; CPUs 4-5 are robot control.
NON_CONTROL_CPUS="${NON_CONTROL_CPUS:-0-1,3,6-31}"
REALTIME_CPUS="${REALTIME_CPUS:-${FCI_IRQ_CPU},${CONTROL_CPUS}}"
GRUB_FILE=/etc/default/grub
# `irqaffinity` does not affect managed MSI-X vectors (notably NVMe blk-mq
# queues).  The managed_irq isolcpus flag is required to keep those vectors
# off the Franka real-time pair once the storage driver creates its queues.
# Isolate the FCI IRQ CPU as well as controller CPUs.  Otherwise a managed
# NVMe MSI-X vector or ordinary host process can remain on the FCI IRQ CPU and
# break libfranka's 1 kHz communication deadline.
REALTIME_ARGS="isolcpus=domain,managed_irq,${REALTIME_CPUS} nohz_full=${REALTIME_CPUS} rcu_nocbs=${REALTIME_CPUS} irqaffinity=${NON_CONTROL_CPUS}"
ROOT_HELPER_IMAGE="${ROOT_HELPER_IMAGE:-franka-vr:humble}"

if [[ ! -f "$GRUB_FILE" ]]; then
  echo "Cannot find $GRUB_FILE" >&2
  exit 1
fi

if grep -Eq "(^|[[:space:]\"])isolcpus=domain,managed_irq,${REALTIME_CPUS}([[:space:]\"]|$)" "$GRUB_FILE" && \
   grep -Eq "(^|[[:space:]\"])nohz_full=${REALTIME_CPUS}([[:space:]\"]|$)" "$GRUB_FILE" && \
   grep -Eq "(^|[[:space:]\"])rcu_nocbs=${REALTIME_CPUS}([[:space:]\"]|$)" "$GRUB_FILE" && \
   grep -Eq "(^|[[:space:]\"])irqaffinity=${NON_CONTROL_CPUS}([[:space:]\"]|$)" "$GRUB_FILE"; then
  echo "CPUs ${REALTIME_CPUS} are already isolated and non-FCI IRQs avoid them at boot."
  exit 0
fi

root_command() {
  if [[ "$EUID" -eq 0 ]]; then
    "$@"
  elif sudo -n true >/dev/null 2>&1; then
    sudo -n "$@"
  elif command -v docker >/dev/null 2>&1 && docker image inspect "$ROOT_HELPER_IMAGE" >/dev/null 2>&1; then
    # Match run_robot.sh's non-interactive host-tuning path.  Joining PID 1's
    # mount namespace makes the command operate on the host filesystem, not
    # on the disposable container's /etc.
    docker run --rm --privileged --pid host "$ROOT_HELPER_IMAGE" \
      nsenter -t 1 -m -p -- "$@"
  else
    echo "Root access is required (passwordless sudo or Docker image $ROOT_HELPER_IMAGE)." >&2
    return 1
  fi
}

echo "This will reserve CPUs ${CONTROL_CPUS} for the franka_robot container, isolate FCI IRQ CPU ${FCI_IRQ_CPU}, and keep boot-time IRQs on ${NON_CONTROL_CPUS}."
echo "The current GRUB configuration will be backed up as ${GRUB_FILE}.franka-vr.bak."

if [[ ! -e "${GRUB_FILE}.franka-vr.bak" ]]; then
  root_command cp "$GRUB_FILE" "${GRUB_FILE}.franka-vr.bak"
else
  echo "Keeping existing backup: ${GRUB_FILE}.franka-vr.bak"
fi

# Add missing arguments and correct stale values independently.  The old
# implementation skipped the entire edit as soon as isolcpus was present,
# leaving irqaffinity absent while still reporting success.
for argument in $REALTIME_ARGS; do
  key=${argument%%=*}
  if grep -Eq "(^|[[:space:]\"])${key}=" "$GRUB_FILE"; then
    root_command sed -i -E \
      "/^GRUB_CMDLINE_LINUX_DEFAULT=/ s#${key}=[^ \"]*#${argument}#" "$GRUB_FILE"
  else
    root_command sed -i -E \
      "/^GRUB_CMDLINE_LINUX_DEFAULT=/ s#\"\$# ${argument}\"#" "$GRUB_FILE"
  fi
done
root_command update-grub

for argument in $REALTIME_ARGS; do
  if ! grep -Eq "(^|[[:space:]\"])${argument}([[:space:]\"]|$)" "$GRUB_FILE"; then
    echo "Failed to write required kernel argument: $argument" >&2
    exit 1
  fi
done

echo
echo "Configuration written. Reboot the host before starting the robot:"
echo "  sudo reboot"
echo "After reboot, verify with: cat /sys/devices/system/cpu/isolated"
