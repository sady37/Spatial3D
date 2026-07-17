# pose — 6844 per-track pose MLP (Phase 2)

Trains the 4-class posture MLP (Stood / Sat / Lying / Falling) that runs on the
6844 People_Tracking firmware and exports it as C the R5F can evaluate directly
(BatchNorm folded into the Linear layers — no TVM, no `pose_model.a`, which is
Cortex-M4 Thumb-only and won't link into the R5F image).

This is an **auxiliary** fall leg — "free pose + fall-motion trigger". The
primary fall decision stays server-side (window + cube-RR). See the memories
`fall-detection-design` and `fall-modular-pipeline`.

## Files
- `dataset.py` — cleaned reproduction of TI's feature extraction from
  `classes.zip`. Drops posz-dead recordings and the velx/accx schema-leak
  features; drops walking; 20 features × 8 frames = 160. Comments explain each
  deviation with the measured defect it fixes.
- `model.py` — the MLP + `fold_bn` (folds each BatchNorm into the following
  Linear) + `folded_forward` (numpy reference the firmware mirrors exactly).
- `train.py` — trains, reports honest grouped-by-file accuracy vs TI's inflated
  random-row split, validates the fold against torch, and (`--out`) exports
  `pose_model.c`.
- `export_c.py` — writes the folded weights as `.rodata.pose_model` C arrays.
- `test_pose.py` — pytest for the fold math (no dataset needed; needs torch).
- `host_test/` — compiles the firmware `pose_mlp.c` + `pose_model.c` natively
  and checks the C forward pass and the full `PoseMlp_process` path against the
  Python reference. Run after any re-export, before the VM build.

## Environment
Intel-mac PyTorch stops at 2.2.2 (needs numpy<2), which conflicts with the main
`pc/.venv` (numpy 2.1). Training uses an isolated venv:

```
python3.12 -m venv .venv-pose
.venv-pose/bin/pip install "numpy<2" "torch==2.2.2" pandas scikit-learn pytest
```

## Retrain + export + validate
```
# unzip TI's dataset somewhere (ti_ref/.../retraining_resources/dataset/classes.zip)
.venv-pose/bin/python -m pose.train \
    --data <classes-dir> \
    --out ../firmware/people_tracking_6844/src/6844/mss/source/pose/pose_model.c

.venv-pose/bin/python -m pytest pose/test_pose.py -q      # fold math
./pose/host_test/run.sh                                    # C == Python end-to-end
```

## Caveat
Weights are trained on TI's 6432 `classes.zip` (8 antennas, TI's mount). Cleaned
of the dataset artifacts, but still 6432 geometry: on 6844 the height axis is the
dominant feature, so calibrate the firmware `poseCfg <enable> <zOffset_cm>` so a
standing person reads posz ≈ +0.33 m. Grouped-by-file accuracy is ~98%, but the
5 subjects are shared between train and test folds (TI recorded only 5 people),
so treat it as an optimistic upper bound.
