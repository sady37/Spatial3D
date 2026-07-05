# Spatial3D — Dev & Debug Setup

Scaffold for the three-layer architecture in [README.md](README.md):
DSP (voxel engine, C) → UART (voxel-map sync) → PC (spatial modeling, Python).

```
dsp/                 DSP layer (C, host-buildable smoke test)
  include/voxel.h    Voxel struct (8B) + grid geometry — mirror of the PC side
  src/voxel.c        occupancy-grid update logic
  src/main.c         gcc-buildable smoke test
pc/                  PC layer (Python 3.12)
  spatial3d/
    voxel.py         Voxel / VoxelGrid, 8B UART wire format
    uart_reader.py   framed voxel-map sync over pyserial
    modeling.py      Open3D point cloud, RANSAC planes, DBSCAN furniture
    simulator.py     synthetic room (runs without radar)
    main.py          entry point
  tests/             pytest (no hardware/Open3D needed)
  .venv/             Python 3.12 venv (Open3D has no 3.14 wheels)
```

## PC layer

```bash
cd pc
.venv/bin/pip install -r requirements.txt      # already done during setup
.venv/bin/python -m spatial3d.main --sim        # synthetic room
.venv/bin/python -m spatial3d.main --sim --viz  # + Open3D viewer
.venv/bin/pytest -q
```

Real hardware: `python -m spatial3d.main --port /dev/tty.usbserial-XXXX`.

## DSP layer

Host build (for logic debugging on the Mac):

```bash
gcc -g -O0 -Wall -Wextra -std=c11 -Idsp/include \
    dsp/src/voxel.c dsp/src/main.c -o dsp/build/voxel_test
./dsp/build/voxel_test
```

On-target builds (AWR6844AOP) require **TI Code Composer Studio + mmWave SDK**,
which are installed and flashed outside VSCode — this repo only carries the
portable voxel logic.

## VSCode

- **Run and Debug** panel → `PC: main (--sim)`, `PC: main (--sim --viz)`,
  `PC: current file`, or `DSP: voxel (gcc)` (see [.vscode/launch.json](.vscode/launch.json)).
- Tasks: `dsp: build`, `pc: pytest` ([.vscode/tasks.json](.vscode/tasks.json)).
- Select the interpreter `pc/.venv/bin/python` if VSCode doesn't auto-detect it.

### Extensions
Installed: Python, Pylance, debugpy. **Missing for the DSP side:** the
C/C++ extension (`ms-vscode.cpptools`) — install it to get IntelliSense and the
`DSP: voxel (gcc)` debug config (uses `lldb`).
