#!/usr/bin/env bash
set -euo pipefail
remote_host="${VLA_REMOTE_HOST:-mac_29}"; port="${VLA_SERVER_PORT:-8093}"; gpu="${VLA_CUDA_VISIBLE_DEVICES:-3}"
remote_project="${VLA_REMOTE_PROJECT:-/mnt/82_store/lyh/VLA/Ours_VQAQ_VLA}"
remote_deploy_script="${VLA_REMOTE_DEPLOY_SCRIPT:-vla-scripts/run_deploy_franka_simulation.sh}"
exec ssh -t "$remote_host" "cd '$remote_project' && PORT='$port' CUDA_VISIBLE_DEVICES='$gpu' exec bash '$remote_deploy_script'"
