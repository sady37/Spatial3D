#!/usr/bin/env bash
# Host-validate the firmware pose MLP without a VM: compile pose_mlp.c +
# pose_model.c natively and check the C forward pass + full PoseMlp_process
# path against the Python reference (pc/pose/model.py). Run after re-exporting
# weights (pose.train --out ...) to catch any C/Python divergence before the
# VM build. Requires the pose venv (pc/.venv-pose) for torch.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$HERE/../../../firmware/people_tracking_6844/src/6844/mss/source/pose"
PC="$HERE/../.."
BUILD="$(mktemp -d)"
trap 'rm -rf "$BUILD"' EXIT

# mach-o rejects the ELF section names; strip/override them for the host build.
sed 's/__attribute__((section(".rodata.pose_model")))//' "$FW/pose_model.c" > "$BUILD/pose_model_host.c"
cc -O2 -DPOSE_TCMB= -I "$FW" "$HERE/pose_host_test.c" "$BUILD/pose_model_host.c" -o "$BUILD/pose_host_test" -lm
cc -O2 -DPOSE_TCMB= -I "$FW" "$HERE/pose_proc_test.c" "$BUILD/pose_model_host.c" -o "$BUILD/pose_proc_test" -lm
echo "host binaries built"

PY="${POSE_PY:-$PC/.venv-pose/bin/python}"
POSE_HOST="$BUILD/pose_host_test" POSE_PROC="$BUILD/pose_proc_test" \
    "$PY" "$HERE/proc_ref.py"
