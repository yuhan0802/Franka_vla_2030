#!/usr/bin/env bash
# Sourced by local launchers. Stop/status must work even if the disk is absent.
export FRANKA_LOG_ROOT="${FRANKA_LOG_ROOT:-/media/mac/Elements1/franka_vr_docker/logs}"
FRANKA_LOG_ROOT="${FRANKA_LOG_ROOT%/}"
case "${1:-run}" in
  stop|STOP|status|server-stop|server-status) ;;
  *)
    if [[ "$FRANKA_LOG_ROOT" == /media/mac/Elements1/* ]] && ! mountpoint -q /media/mac/Elements1; then
      echo 'Log disk /media/mac/Elements1 is not mounted; refusing to write logs to the system disk.' >&2
      return 1
    fi
    mkdir -p "$FRANKA_LOG_ROOT" || return 1
    ;;
esac
