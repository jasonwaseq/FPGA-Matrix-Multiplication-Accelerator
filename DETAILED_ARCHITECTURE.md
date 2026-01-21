# DVS Gesture Detector - Detailed Architecture

## System Overview

This is an event-driven FPGA gesture recognition accelerator that processes Dynamic Vision Sensor (DVS) events and classifies them into 4 directional gestures using **matrix multiplication**.

```
┌────────────────────────────────────────────────────────────────────┐
│                 dvs_gesture_detector_top                           │
│                                                                     │
│  Events In    ┌─────────────┐      ┌─────────────┐      ┌────────┐│
│  (x,y,pol)───▶│    Event    │─────▶│  Activity   │─────▶│ Matmul ││
│               │   Buffer    │      │    Map      │      │ Class  ││
│               │   (FIFO)    │      │  (8×8 RAM)  │      │(4 cores)
│               └─────────────┘      └──────┬──────┘      │        ││
│                                           │ decay       │        ││
│        ┌────────────────────────────────┬─┘             └────┬───┘│
│        │ Periodic Triggers (every 1000  │                    │    │
│        │ cycles)                        │              Gesture   │
│        │ - decay_trigger                │              Output    │
│        │ - classify_trigger             │              (0-3)     │
│        └────────────────────────────────┘                    │    │
│                                                              ▼    │
└────────────────────────────────────────────────────────────────────┘
```

---

## 1. Event Buffer (dvs_event_buffer)

### Purpose
Buffers incoming DVS events to smooth burst arrivals and provide ready/valid handshaking.

### Interface
- **Input**: `dvs_event_valid_i`, `dvs_x_i`, `dvs_y_i`, `dvs_polarity_i`
- **Output**: Buffered events with `event_buf_valid`, event coordinates, polarity
- **Handshake**: `dvs_ready_o` (backpressure)

### Operation
- FIFO with depth `2^EVENT_BUFFER_DEPTH_LOG2` (256 events default)
- Absorbs event bursts without dropping data
- Passes events to activity map with ready/valid protocol

### Why It's Needed
DVS cameras produce burst-like events during fast motion. The FIFO decouples event input from activity map processing, allowing the map to take multiple cycles to process each event.

---

## 2. Activity Map (activity_map)

### Purpose
Maintains a 2D map of DVS activity with **exponential decay** to emphasize recent events.

### Data Structure
- **8×8 signed counter array** (64 pixels total)
- **8-bit counters** per pixel (range: -128 to +127)
- Stored in single-port synchronous RAM

### Parameters
```
WIDTH_P = 8        // pixels wide
HEIGHT_P = 8       // pixels tall
COUNTER_WIDTH_P = 8 // bits per pixel
```

### State Machine

```
┌──────────────────────────────────────────────────────────────┐
│                        S_IDLE                                │
│  - Wait for event OR decay trigger                           │
│  - Accept new events                                         │
│  - Ready for classifier reads                                │
└──────────┬───────────────────────────────┬──────────────────┘
           │ event_valid_i                  │ decay_trigger_i
           ▼                                ▼
        S_UPDATE                        S_DECAY
     (1 cycle)                      (64 cycles total)
  - Read current pixel            - Loop through 64 pixels
  - Inc/Dec based on polarity     - Read pixel: mem_rd_data
  - Write back saturated          - Right-shift: >>1
  - Return to IDLE                - Write back
                                  - Return to IDLE
```

### Event Processing (S_UPDATE)
```systemverilog
if (event_polarity_i == 1)  // ON event
  new_value = current_value + 1  (saturated at +127)
else                        // OFF event
  new_value = current_value - 1  (saturated at -128)
```

### Decay Processing (S_DECAY)
- Iterates through all 64 pixels
- Each pixel is **right-shifted by 1** (divide by 2, round toward zero)
- Exponential decay: `new_value = value >> 1`
- Asymptotically approaches zero, keeps old activity low
- Takes 64 cycles to complete

### Address Calculation
```
pixel_address = y * WIDTH_P + x = y * 8 + x
```

### Key Features
- **Single-port RAM** = only one operation (read or write) per cycle
- **Saturating arithmetic** = prevents overflow on inc/dec
- **Decay on demand** = triggered by top-level every 1000 cycles
- **Parallel read** = classifier can read while events process

---

## 3. Matrix Multiplication Core (matmul_core)

### Purpose
Implements a **streaming Multiply-Accumulate (MAC)** unit for computing dot products.

### Architecture
```
Input Stream    Multiplication    Accumulation      Output
                                  (register)
a_i ──┐                                          result_o
      ├─────▶ [×] ──────────┐                    (ACC_WIDTH_P bits)
b_i ──┘                      │
                        ┌─────┴─────┐
                        │  +  ACC   │ ◀─── Feedback
                        └─────┬─────┘
                              │ F/F
                          acc_clear_i
```

### Parameters
```
WIDTH_P = 8              // input width (activity or weight)
MULT_WIDTH_P = 16        // a_i * b_i = 16 bits max
ACC_WIDTH_P = 32         // accumulator width
```

### Operation
```systemverilog
On each valid_i:
  mult_result = a_i * b_i  (16-bit signed)
  accumulator <= accumulator + mult_result  (32-bit)
```

### Key Features
- **Always ready** = no backpressure
- **Streaming** = accepts 1 input per cycle
- **Pipelined multiplication** = synthesis tool infers DSP blocks on FPGA
- **Clearable accumulator** = `acc_clear_i` resets to 0
- **Signed arithmetic** = handles positive and negative values

### Throughput
- **1 MAC per cycle** per core
- With 4 cores: **4 MACs/cycle** (parallel dot product)

---

## 4. Matmul Gesture Classifier (matmul_gesture_classifier)

### Purpose
Classifies the 2D activity map into one of 4 gestures using **learned weight matrices** and matrix multiplication.

### Weight Matrix Structure
```
Gesture  │  Pixel 0  Pixel 1  ...  Pixel 63
─────────┼────────────────────────────────
0 (UP)   │   +2       +2     ...    -2
1 (DOWN) │   -2       -2     ...    +2
2 (LEFT) │   +2       +2     ...    -2
3 (RIGHT)│   -2       -2     ...    +2
```

- **4 gestures × 64 pixels** weight matrix
- Each weight: **8-bit signed** (-128 to +127)
- **Directional kernels** initialized at synthesis:
  - UP: positive weights in top half, negative in bottom
  - DOWN: inverted
  - LEFT: positive left, negative right
  - RIGHT: inverted

### State Machine

```
┌─────────────────────────────────────────────────────┐
│              S_IDLE                                 │
│  - Wait for classify_trigger_i                      │
│  - Clear all 4 accumulators                         │
└──────────────────┬──────────────────────────────────┘
                   │ classify_trigger_i = 1
                   ▼
┌─────────────────────────────────────────────────────┐
│              S_COMPUTE                              │
│  - Loop pixel_idx = 0 to 63                         │
│  - For each pixel:                                  │
│    - Request activity map read: map_read_addr_o     │
│    - Feed to all 4 matmul cores:                    │
│      * a_i = activity_map[pixel_idx]                │
│      * b_i = weight[gesture][pixel_idx]             │
│    - Cores accumulate: score += activity × weight   │
└──────────────────┬──────────────────────────────────┘
                   │ pixel_idx == 64
                   ▼
┌─────────────────────────────────────────────────────┐
│              S_DECIDE                               │
│  - Find argmax(scores[0:3])                         │
│  - Output winning gesture + confidence              │
└──────────────────┬──────────────────────────────────┘
                   │ return to S_IDLE
                   └──────────────────────────────────┘
```

### Classification Computation

For each gesture `g` and pixel `i`:
```
score[g] = Σ(activity_map[i] × weight[g][i])  for i = 0 to 63
```

**In parallel (4 matmul cores)**:
```
Core 0: score_UP    = activity · weights_UP
Core 1: score_DOWN  = activity · weights_DOWN
Core 2: score_LEFT  = activity · weights_LEFT
Core 3: score_RIGHT = activity · weights_RIGHT
```

### Output
```
gesture_idx_o = argmax(score[0:3])
confidence_score_o = max(score[0:3])
```

### Example (Assuming Activity in Top Half):
```
Scenario: User flicks DVS upward
  ├─ Top pixels:    activity ≈ +100 (many ON events)
  ├─ Bottom pixels: activity ≈ -50 (some OFF events)
  │
  └─ Score Computation:
     ├─ score_UP    = 100*(+2) + 100*(+2) + ... + (-50)*(-2) = HIGH ✓
     ├─ score_DOWN  = 100*(-2) + 100*(-2) + ... + (-50)*(+2) = LOW
     ├─ score_LEFT  = mixed horizontal activity = MEDIUM
     └─ score_RIGHT = mixed horizontal activity = MEDIUM
  │
  └─ Output: gesture_idx = 0 (UP), confidence = score_UP
```

### Latency
- **S_IDLE** → **S_COMPUTE**: 0 cycles
- **S_COMPUTE**: 64 cycles (one per pixel)
- **S_DECIDE**: 1 cycle
- **Total classification latency**: 65 cycles

---

## 5. Top-Level Orchestration (dvs_gesture_detector_top)

### Periodic Control

Every **CLASSIFY_INTERVAL_P** cycles (default: 1000):
```
Cycle 1000:  decay_trigger_reg = 1, classify_trigger_reg = 1
Cycle 1001:  Activity map begins decay (64 cycles)
             Classifier begins reading map (64 cycles)
Cycle 1065:  Both operations complete, gesture output valid
Cycle 1001-1099: Normal event processing resumes
Cycle 2000:  Next cycle boundary
```

### Interval Counter
```systemverilog
if (interval_counter >= CLASSIFY_INTERVAL_P) begin
  interval_counter <= 0
  decay_trigger_reg <= 1
  classify_trigger_reg <= 1
end else begin
  interval_counter <= interval_counter + 1
end
```

### Why 1000 Cycles?
- DVS at 30,000 events/second → ~1µs per event on average
- 1000 cycles @ 100 MHz = 10 µs window
- **Captures motion snapshots** at 100 Hz intervals
- **Allows decay** to emphasize recent motion

---

## Data Flow Example: UP Gesture

### Timeline
```
t=0:       Reset
t=1-500:   Events arrive in top half (y < 4)
           Activity map pixels [0:31] increment to ~100
           Bottom pixels [32:63] stay near 0

t=1000:    Decay & Classify triggers fire

t=1001-1064:
           Decay Phase (Activity Map):
             Pixel 0: 100 >> 1 = 50
             Pixel 1: 100 >> 1 = 50
             ...
             All pixels: value >> 1
           
           Compute Phase (Classifier):
             Loop pixel 0-63:
               Core 0 (UP):    accumulates activity[i] * weights_UP[i]
               Core 1 (DOWN):  accumulates activity[i] * weights_DOWN[i]
               Core 2 (LEFT):  accumulates activity[i] * weights_LEFT[i]
               Core 3 (RIGHT): accumulates activity[i] * weights_RIGHT[i]

t=1065:    Decide Phase:
             score_UP    = 50*2 + 50*2 + ... = ~3000 (HIGH)
             score_DOWN  = 50*(-2) + 50*(-2) + ... = ~-3000 (LOW)
             score_LEFT  ≈ 0 (balanced horizontal)
             score_RIGHT ≈ 0
             
             gesture_valid_o = 1
             gesture_idx_o = 0 (UP)
             confidence_score_o = 3000

t=1066:    Events resume processing, gesture output valid
t=1001-2000: Accumulate new events, decay continues periodically
```

---

## Key Design Decisions

### 1. **8×8 Spatial Resolution**
- ✅ Small enough for low latency (64 pixels)
- ✅ Large enough for spatial discrimination
- ✅ Fits nicely in typical DVS sensors (128×128)

### 2. **8-bit Activity Counters**
- ✅ Compact storage (64 bytes for entire map)
- ✅ Range: -128 to +127 (sufficient for event density)
- ✅ Saturating arithmetic prevents overflow

### 3. **Right-shift Decay**
- ✅ Hardware efficient (no multiplier needed)
- ✅ Exponential falloff is mathematically sound
- ✅ No division needed (integer only)

### 4. **Parallel Matmul Cores**
- ✅ 4 cores = 4 simultaneous dot products
- ✅ Full gesture classification per cycle
- ✅ Scales easily to more gestures (add cores)

### 5. **Periodic Classification**
- ✅ Fixed latency = predictable output
- ✅ Decay cleaning = removes noise
- ✅ 100 Hz gesture updates (1000 cycles @ 100 MHz)

### 6. **Single-Port RAM Activity Map**
- ✅ Minimal area (1 read + 1 write port needed)
- ✅ Can multiplex: events write, classifier reads
- ✅ Decay consumes 64 cycles (manageable)

---

## Performance Metrics

| Metric | Value |
|--------|-------|
| **Input Event Rate** | Up to 1 event/cycle (100+ MHz) |
| **Classification Latency** | 65 cycles |
| **Classification Rate** | Every 1000 cycles |
| **Output Latency** | 1 cycle (registered output) |
| **Memory** | 64 × 8 bits = 512 bits (activity map) + 256 × 8 bits = 2 KB (weights) |
| **DSP Blocks** | 4 × multiply + 4 × accumulate (using 4 matmul cores) |
| **LUT Usage** | ~500-1000 LUTs (estimate, depends on synthesis) |

---

## Extension Ideas

1. **Larger sensor**: Change `WIDTH_P`, `HEIGHT_P` (e.g., 16×16)
2. **More gestures**: Add cores (6 or 8 directions)
3. **Learned weights**: Load from external memory instead of hardcoded
4. **Adjustable decay**: Make decay rate parameterizable
5. **Temporal filtering**: Multiple activity maps per time window
6. **Confidence threshold**: Only output if `confidence > threshold`
7. **Movement tracking**: Combine with velocity estimation
