# DVS Gesture Detector - Simplified Architecture

## Signal Flow

```
                                    ┌─────────────────────────────────┐
                                    │  dvs_gesture_detector_top      │
                                    │                                 │
  DVS Event Stream                  │                                 │
  ─────────────────────────────────▶│  ┌──────────────────┐          │
  (x, y, polarity)                  │  │  dvs_event_buffer │          │
                                    │  │  (FIFO)          │          │
                                    │  └────────┬─────────┘          │
                                    │           │                     │
                                    │           ▼                     │
                                    │  ┌──────────────────┐          │
                                    │  │  activity_map    │          │
                                    │  │  - 8x8 signed    │          │
                                    │  │  - increment/dec │          │
                                    │  │  - decay (>>1)   │          │
                                    │  └────────┬─────────┘          │
                                    │           │                     │
           Periodic                 │           │ read                │
           triggers ────────────────┼───────────┤                     │
           (every 1000 cycles)      │           │                     │
                                    │           ▼                     │
                                    │  ┌──────────────────────────┐  │
                                    │  │ simple_gesture_classifier│  │
                                    │  │ - sum top/bot            │  │
                                    │  │ - sum left/right         │  │
                                    │  │ - pick strongest         │  │
                                    │  └────────┬─────────────────┘  │
                                    │           │                     │
                                    │           ▼                     │
  Gesture Output ◀──────────────────┼───────────────────────────────│
  (0=UP, 1=DOWN, 2=LEFT, 3=RIGHT)   │   gesture_idx[1:0]             │
  + confidence score                │   confidence_score             │
                                    └─────────────────────────────────┘
```

## Modules

### 1. **dvs_event_buffer** (FIFO)
- Buffers incoming DVS events
- Smooths burst arrivals
- Ready/valid handshake

### 2. **activity_map**
- 8×8 array of signed 8-bit counters
- **Event processing**: increment (polarity=1) or decrement (polarity=0)
- **Decay operation**: global right-shift (>>1) on trigger
- **Read interface**: parallel access for classifier

### 3. **simple_gesture_classifier**
- **Scan phase**: reads entire activity map
- **Accumulation**: 
  - Top half vs. bottom half → vertical bias
  - Left half vs. right half → horizontal bias
- **Decision**: picks dominant direction (largest magnitude)
- **Output**: gesture index + confidence

### 4. **dvs_gesture_detector_top**
- Integrates all modules
- Periodic trigger every 1000 cycles:
  1. Decay the activity map
  2. Run classifier
- Outputs gesture when valid

## Timing

```
Cycle 0-999:    Accept events, update activity map
Cycle 1000:     Decay trigger + classify trigger
Cycle 1001-1064: Classifier scans map (64 pixels)
Cycle 1065:     Gesture output valid
Cycle 1066-1999: Accept events, update map
Cycle 2000:     Next decay/classify cycle
...
```

## Test Strategy

Each cocotb test:
1. Reset the design
2. Send events concentrated in one spatial region (top/bottom/left/right)
3. Wait for classification interval (~1000 cycles)
4. Check `gesture_idx` and `confidence_score`

Tests verify:
- UP: events in rows 0-3
- DOWN: events in rows 4-7
- LEFT: events in columns 0-3
- RIGHT: events in columns 4-7
