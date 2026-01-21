# DVS Gesture Detector - Matrix Multiplication Accelerator

FPGA-based gesture recognition using matrix multiplication for Dynamic Vision Sensor (DVS) event streams.

## Features

- **Event-driven processing**: Accepts DVS events (x, y, polarity)
- **Activity map with decay**: Single 2D map with periodic decay
- **Matrix multiplication classifier**: Learned weight matrices for gesture detection
- **4 Matmul cores**: Parallel classification for UP, DOWN, LEFT, RIGHT
- **Cocotb testbench**: Python-based simulation tests

## Architecture

```
DVS Events → Event Buffer → Activity Map (with decay) → Matmul Classifier (4×64 weights) → Gesture Output
```

### Activity Map
- Single 2D array of signed counters (8×8 pixels)
- Increment/decrement on each event based on polarity
- Global decay: periodic right-shift of all values

### Matmul Classifier
- **4 parallel matmul cores** (one per gesture class)
- **Learned weight matrix**: 4 gestures × 64 pixels (directional kernels)
- Each core computes: `score[g] = Σ(activity[i] × weight[g][i])`
- Outputs gesture with maximum score + confidence

## Quick Start

### Simulation

```bash
# Run cocotb tests (cocotb/pytest installed in devcontainer)
make sim
```

### Synthesis (iCEBreaker)

```bash
make synth
```

## Parameters

- `WIDTH_P`, `HEIGHT_P`: Sensor resolution (default: 8×8)
- `COUNTER_WIDTH_P`: Activity counter bit width (default: 8)
- `CLASSIFY_INTERVAL_P`: Cycles between classifications (default: 1000)

## Gesture Mapping

- `0`: UP (more activity in top half)
- `1`: DOWN (more activity in bottom half)
- `2`: LEFT (more activity in left half)
- `3`: RIGHT (more activity in right half)

## Directory Structure

```
├── rtl/                              # RTL source files
│   ├── ram_1r1w_sync.sv             # RAM primitive
│   ├── fifo_1r1w_sync.sv            # FIFO primitive
│   ├── dvs_event_buffer.sv          # Event input buffer
│   ├── activity_map.sv              # 2D activity map with decay
│   ├── matmul_core.sv               # MAC unit for matrix multiply
│   ├── matmul_gesture_classifier.sv # Matmul-based classifier (4 cores)
│   └── dvs_gesture_detector_top.sv  # Top-level module
├── tb/                               # Cocotb testbenches
│   ├── dvs_gesture_testbench.py     # Python tests
│   └── Makefile                     # Cocotb build
└── Makefile                          # Top-level build
```
