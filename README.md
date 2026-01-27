# DVS Gesture Accelerator with Temporal Voxel Binning

FPGA-based gesture recognition accelerator for Dynamic Vision Sensor (DVS) events on iCE40 UP5K.

## Architecture Overview

This accelerator processes DVS events using a temporal voxel representation with the following pipeline:

```
DVS Events (320×320) → Spatial Compression (16×16) → Temporal Binning (8 bins)
                    → Double Ring Buffer → Feature Extraction → Matrix Multiply → Gesture
```

### Key Features

- **Spatial Compression**: 320×320 sensor → 16×16 grid (20×20 pixel pooling)
- **Temporal Binning**: 400µs sliding window divided into 8 × 50µs bins
- **Double Ring Buffer**: Continuous event accumulation without blocking classification
- **Polarity Channels**: Separate ON/OFF event tracking per voxel
- **Matrix Multiplication**: Centroid-based motion estimation with argmax classification

### Memory Organization

- **Voxel Tensor**: 16×16 spatial × 8 temporal × 2 polarity = 4096 voxels
- **Double Buffered**: 2 rings × 4096 voxels × 4-bit counters = 32 Kbit total
- **Input FIFO**: 8-deep buffer for burst event handling

### Timing

| Parameter | Value |
|-----------|-------|
| Clock | 12 MHz |
| Bin Period | 50 µs (600 cycles) |
| Window Period | 400 µs (4800 cycles) |
| Ring Swap | Every 400 µs |

## UART Protocol

**Configuration**: 115200 baud, 8N1

### Send Events to FPGA (5 bytes per event)

| Byte | Contents |
|------|----------|
| 0 | X_HI: `X[8]` (bit 0) |
| 1 | X_LO: `X[7:0]` |
| 2 | Y_HI: `Y[8]` (bit 0) |
| 3 | Y_LO: `Y[7:0]` |
| 4 | POL: Polarity (0=OFF, 1=ON) |

X, Y coordinates range from 0-319.

### Receive from FPGA (1 byte)

**Gesture Response**: `0xA0 | gesture`
- 0xA0 = UP
- 0xA1 = DOWN  
- 0xA2 = LEFT
- 0xA3 = RIGHT

**Status Response**: `0xB0 | bin`
- Current temporal bin (0-7)

### Special Commands

| Send | Response | Description |
|------|----------|-------------|
| 0xFF | 0x55 | Echo test (verify UART) |
| 0xFE | 0xBx | Status query (current bin) |

## File Structure

```
rtl/
├── uart_gesture_top.sv    # Top-level UART wrapper
├── dvs_gesture_accel.sv   # Main accelerator (FIFO, voxels, classifier)
├── uart_rx.sv             # UART receiver
└── uart_tx.sv             # UART transmitter

tb/
├── Makefile               # Cocotb simulation makefile
└── test_dvs_gesture.py    # Comprehensive cocotb testbench

synth/
├── Makefile               # Synthesis makefile (Yosys + nextpnr)
└── icebreaker.pcf         # Pin constraints for iCEBreaker

validate_ice40.py          # Hardware validation script
```

## Quick Start

### Simulate with Cocotb

```bash
cd tb
make clean
make
```

This runs all tests including:
- UART echo test
- Status query test
- Coordinate range test
- All four gesture tests (UP, DOWN, LEFT, RIGHT)
- Burst event handling
- Temporal binning verification

### Synthesize for iCE40

```bash
cd synth
make clean
make
```

### Program iCEBreaker

```bash
cd synth
make prog
```

### Validate on Hardware

```bash
python3 validate_ice40.py /dev/ttyUSB0
```

Options:
- `-v`: Verbose output
- `-n 500`: Use 500 events per gesture test
- `--echo-only`: Quick UART connectivity test
- `--reset`: Toggle DTR to reset FPGA

## Resource Usage (iCE40 UP5K)

| Resource | Usage | Available |
|----------|-------|-----------|
| Logic Cells | ~3000 | 5280 |
| Block RAM | ~4 KB | 120 KB |
| I/O | 4 | 39 |

## Gesture Detection Algorithm

1. **Spatial Compression**: Each event coordinate is divided by 20 to map 320×320 → 16×16
2. **Temporal Binning**: Events are placed in bins based on internal timing (50µs bins)
3. **Double Buffering**: One ring accumulates while the other is read for classification
4. **Feature Extraction**: Compute weighted centroids for early bins (0-3) vs late bins (4-7)
5. **Motion Estimation**: `delta = late_centroid - early_centroid`
6. **Classification**: Matrix multiply equivalent - compare |dx| vs |dy| and their signs

## Debug LEDs

| LED | Function |
|-----|----------|
| led_heartbeat | Blinks at ~3Hz when FPGA running |
| led_gesture_valid | Pulses when gesture detected |
