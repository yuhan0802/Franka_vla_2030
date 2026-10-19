#!/usr/bin/env bash
# Keep high-rate, non-FCI device interrupts away from the FR3 control cores.
# The FCI NIC's IRQ is intentionally not changed here: run_robot.sh pins it
# to its dedicated network CPU before each real-robot launch.
set -euo pipefail

CONTROL_CPUS="${CONTROL_CPUS:-4,5}"
NON_CONTROL_CPUS="${NON_CONTROL_CPUS:-0-3,6-31}"
IRQ_DEVICE_PATTERN="${IRQ_DEVICE_PATTERN:-nvme|iwlwifi|thunderbolt}"

while IFS= read -r irq; do
  [[ -n "$irq" && -w "/proc/irq/$irq/smp_affinity_list" ]] || continue
  # NVMe MSI-X queues can be kernel-managed and reject runtime affinity
  # writes.  They are handled by the irqaffinity kernel argument at boot;
  # do not make the remaining valid device rules fail because of one queue.
  if ! echo "$NON_CONTROL_CPUS" > "/proc/irq/$irq/smp_affinity_list"; then
    echo "IRQ $irq is kernel-managed; leave it for boot-time irqaffinity" >&2
    continue
  fi
  effective="$(cat "/proc/irq/$irq/effective_affinity_list")"
  case ",$effective," in
    *,4,*|*,5,*)
      echo "IRQ $irq remains on FR3 control CPUs $CONTROL_CPUS; effective=$effective" >&2
      ;;
  esac
  echo "Pinned non-FCI IRQ $irq to CPUs $effective"
done < <(grep -E "$IRQ_DEVICE_PATTERN" /proc/interrupts | cut -d: -f1 | tr -d ' ')
