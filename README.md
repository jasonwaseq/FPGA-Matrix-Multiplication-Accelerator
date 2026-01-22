# DVS Gesture Accelerator for Ice40 FPGA

A lightweight matrix multiplication accelerator for classifying gesture movements from Dynamic Vision Sensor (DVS) event data. Designed for the iCE40 UP5K FPGA (iCEBreaker board). Includes UART interface for testing without real DVS hardware.

## Overview

This accelerator processes DVS events (x, y, polarity) and classifies them into 4 gestures: **UP**, **DOWN**, **LEFT**, **RIGHT**.

---

## How the Accelerator Works

The accelerator implements a **3-stage pipeline** that converts raw DVS event streams into gesture classifications using matrix multiplication:

```
┌─────────────┐    ┌─────────────────┐    ┌────────────────┐    ┌──────────┐
│  DVS Events │───▶│ Event           │───▶│ Gesture        │───▶│ Gesture  │
│  (x,y,pol)  │    │ Accumulator     │    │ Classifier     │    │ Output   │
│             │    │ (delta vector)  │    │ (matrix mult)  │    │ (0-3)    │
└─────────────┘    └─────────────────┘    └────────────────┘    └──────────┘
```

### Stage 1: Event Accumulation (`event_accumulator.sv`)

DVS cameras output asynchronous events whenever a pixel detects brightness change. Each event contains:
- **x, y**: Pixel coordinates (0-127 for DVS128)
- **polarity**: ON (+1) for brightness increase, OFF (-1) for decrease

The accumulator computes a **weighted center of mass** over a window of N events (default: 100):

```
For each event:
    sum_x += x × polarity    (polarity = +1 or -1)
    sum_y += y × polarity
    event_count++
```

**Why this works**: When you swipe RIGHT, ON events appear on the right side (+x) and OFF events appear on the left side (-x). The weighted sum naturally becomes positive for x. Similarly for other directions.

After N events, the accumulator computes the **movement delta**:

```
delta_x = sum_x - previous_sum_x
delta_y = sum_y - previous_sum_y
```

This delta vector `[Δx, Δy]` encodes the direction and magnitude of movement.

### Stage 2: Matrix Multiplication (`gesture_classifier.sv`)

The delta vector is multiplied by a **4×2 weight matrix** to produce 4 gesture scores:

```
┌──────────────┐   ┌─────────┐   ┌─────┐
│ score_up     │   │  0   64 │   │ Δx  │
│ score_down   │ = │  0  -64 │ × │ Δy  │
│ score_left   │   │-64    0 │   └─────┘
│ score_right  │   │ 64    0 │
└──────────────┘   └─────────┘
     4×1              4×2        2×1
```

**Expanded matrix multiplication:**
```
score_up    = ( 0 × Δx) + (+64 × Δy) =  64·Δy
score_down  = ( 0 × Δx) + (-64 × Δy) = -64·Δy
score_left  = (-64 × Δx) + ( 0 × Δy) = -64·Δx
score_right = (+64 × Δx) + ( 0 × Δy) =  64·Δx
```

**Weight matrix design rationale:**
| Gesture | Condition | Weight Row | Score Formula |
|---------|-----------|------------|---------------|
| UP      | Δy > 0    | `[ 0, +64]` | +64·Δy (high when moving up) |
| DOWN    | Δy < 0    | `[ 0, -64]` | -64·Δy (high when Δy negative) |
| LEFT    | Δx < 0    | `[-64, 0]` | -64·Δx (high when Δx negative) |
| RIGHT   | Δx > 0    | `[+64, 0]` | +64·Δx (high when moving right) |

The weight value ±64 (2^6) is chosen to:
1. Provide good resolution without overflow
2. Be efficiently implemented as shifts in hardware

### Stage 3: Argmax Classification

The final stage compares all 4 scores and outputs the gesture with the **highest score**:

```verilog
if (score_up >= score_down && score_up >= score_left && score_up >= score_right)
    gesture = UP;
else if (score_down >= score_left && score_down >= score_right)
    gesture = DOWN;
else if (score_left >= score_right)
    gesture = LEFT;
else
    gesture = RIGHT;
```

**Example walkthrough:**
1. User swipes RIGHT → Events cluster at +x positions
2. Accumulator: Δx = +500, Δy = +20
3. Matrix multiply:
   - score_up = 64 × 20 = 1,280
   - score_down = -64 × 20 = -1,280
   - score_left = -64 × 500 = -32,000
   - score_right = 64 × 500 = **32,000** ← highest!
4. Argmax → **RIGHT** (gesture = 3)


## UART Protocol

**Baud Rate**: 115200, 8N1  
**Clock**: 12 MHz (iCEBreaker onboard oscillator)

### Send Event to FPGA (4 bytes per event):
| Byte | Description                |
|------|----------------------------|
| 0    | X coordinate (0-127)       |
| 1    | Y coordinate (0-127)       |
| 2    | Polarity (1=ON, 0=OFF)     |
| 3    | Timestamp (unused, send 0) |

### Receive from FPGA (1 byte when gesture detected):
| Bits  | Description                            |
|-------|----------------------------------------|
| [7:4] | Marker (0xA)                           |
| [1:0] | Gesture: 0=UP, 1=DOWN, 2=LEFT, 3=RIGHT |

## Quick Start

### 1. Simulation (cocotb + Icarus Verilog)
```bash
cd tb
make
```

### 2. Synthesis (Yosys + nextpnr-ice40)
```bash
cd synth
make
```

### 3. Program iCEBreaker
```bash
cd synth
make prog
```

### 4. Validate on Hardware
```bash
python3 validate_ice40.py /dev/ttyUSB1
```
