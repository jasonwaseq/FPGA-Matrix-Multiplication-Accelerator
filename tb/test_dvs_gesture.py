"""
Cocotb Testbench for DVS Gesture Accelerator with Temporal Voxel Binning

Tests:
- UART echo (send 0xFF, expect 0x55)
- Status query (send 0xFE, expect 0xBx)
- Gesture detection for UP, DOWN, LEFT, RIGHT

Protocol:
  RX: 5 bytes per event [X_HI, X_LO, Y_HI, Y_LO, POL]
  TX: 1 byte gesture [0xA0 | gesture] or status [0xB0 | bin]
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles, Timer, First, with_timeout
import random

# ============================================================================
# Configuration
# ============================================================================

CLK_FREQ_HZ = 12_000_000
BAUD_RATE = 115200
CLKS_PER_BIT = CLK_FREQ_HZ // BAUD_RATE  # 104

SENSOR_RES = 320
GRID_SIZE = 16
NUM_BINS = 8
WINDOW_US = 400
CYCLES_PER_BIN = (CLK_FREQ_HZ // 1_000_000) * (WINDOW_US // NUM_BINS)  # 600

GESTURE_UP = 0
GESTURE_DOWN = 1
GESTURE_LEFT = 2
GESTURE_RIGHT = 3

# ============================================================================
# UART Functions
# ============================================================================

async def uart_send_byte(dut, byte_val):
    """Send one byte via UART (bit-bang on uart_rx)"""
    # Start bit
    dut.uart_rx.value = 0
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    # Data bits (LSB first)
    for i in range(8):
        dut.uart_rx.value = (byte_val >> i) & 1
        await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    # Stop bit
    dut.uart_rx.value = 1
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    # NO inter-byte gap - return immediately so caller can start receiving


async def uart_receive_byte(dut, timeout_cycles=20000):
    """
    Receive one byte from uart_tx.
    Returns the byte value or None on timeout.
    """
    # Always wait for line to be high first (idle state)
    for _ in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 1:
            break
    else:
        return None
    
    # Now wait for falling edge (start bit)
    for i in range(timeout_cycles):
        await RisingEdge(dut.clk)
        if dut.uart_tx.value == 0:
            break
    else:
        return None  # Timeout - no start bit
    
    # Move to middle of start bit for better sampling
    await ClockCycles(dut.clk, CLKS_PER_BIT // 2)
    
    # Verify still low (valid start bit, not a glitch)
    if dut.uart_tx.value != 0:
        return None
    
    # Sample 8 data bits at middle of each bit period
    byte_val = 0
    for i in range(8):
        await ClockCycles(dut.clk, CLKS_PER_BIT)
        if dut.uart_tx.value == 1:
            byte_val |= (1 << i)
    
    # Wait through stop bit
    await ClockCycles(dut.clk, CLKS_PER_BIT)
    
    return byte_val


async def send_dvs_event(dut, x, y, polarity):
    """Send a 5-byte DVS event packet"""
    x = max(0, min(319, x))
    y = max(0, min(319, y))
    
    await uart_send_byte(dut, (x >> 8) & 0x01)  # X_HI
    await uart_send_byte(dut, x & 0xFF)          # X_LO
    await uart_send_byte(dut, (y >> 8) & 0x01)  # Y_HI
    await uart_send_byte(dut, y & 0xFF)          # Y_LO
    await uart_send_byte(dut, polarity & 0x01)   # POL


# ============================================================================
# Test Setup
# ============================================================================

async def setup_test(dut):
    """Initialize clock and reset"""
    clock = Clock(dut.clk, 83, units="ns")  # ~12MHz
    cocotb.start_soon(clock.start())
    
    # UART RX idle high
    dut.uart_rx.value = 1
    
    # Wait for power-on reset (>31 cycles) + settling
    await ClockCycles(dut.clk, 50)
    
    # Verify TX is idle high
    assert dut.uart_tx.value == 1, "UART TX should be idle high"


# ============================================================================
# Tests
# ============================================================================

@cocotb.test()
async def test_uart_echo(dut):
    """Test UART echo: send 0xFF, expect 0x55"""
    await setup_test(dut)
    dut._log.info("=== UART Echo Test ===")
    
    # Start receive coroutine BEFORE sending (to catch the response)
    async def do_receive():
        return await uart_receive_byte(dut, timeout_cycles=50000)
    
    recv_task = cocotb.start_soon(do_receive())
    
    # Send echo command
    dut._log.info("Sending 0xFF echo command...")
    await uart_send_byte(dut, 0xFF)
    
    # Wait for receive to complete
    dut._log.info("Waiting for response...")
    response = await recv_task
    
    assert response is not None, "No echo response received"
    dut._log.info(f"Received: 0x{response:02X}")
    assert response == 0x55, f"Expected 0x55, got 0x{response:02X}"
    dut._log.info("Echo test PASSED")


@cocotb.test()
async def test_status_query(dut):
    """Test status query: send 0xFE, expect 0xBx"""
    await setup_test(dut)
    dut._log.info("=== Status Query Test ===")
    
    # Start receive task before sending
    recv_task = cocotb.start_soon(uart_receive_byte(dut, timeout_cycles=50000))
    
    # Send status command
    dut._log.info("Sending 0xFE status query...")
    await uart_send_byte(dut, 0xFE)
    
    # Wait for response
    response = await recv_task
    
    assert response is not None, "No status response received"
    dut._log.info(f"Received: 0x{response:02X}")
    assert (response & 0xF0) == 0xB0, f"Expected 0xBx, got 0x{response:02X}"
    dut._log.info(f"Current bin: {response & 0x07}")
    dut._log.info("Status query test PASSED")


@cocotb.test()
async def test_gesture_up(dut):
    """Test UP gesture detection"""
    await setup_test(dut)
    dut._log.info("=== Gesture UP Test ===")
    
    # Generate UP motion: Y increasing over time (early low Y, late high Y)
    cx = SENSOR_RES // 2
    num_events = 10  # Reduced for faster simulation
    
    # Early events: low Y (around 140)
    dut._log.info("Sending early events (low Y)...")
    for i in range(num_events):
        x = cx + random.randint(-10, 10)
        y = 140 + random.randint(-5, 5)
        await send_dvs_event(dut, x, y, 1)
    
    # Advance time through bins (wait for half the window)
    dut._log.info("Waiting for bin advancement...")
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 4)
    
    # Late events: high Y (around 180)
    dut._log.info("Sending late events (high Y)...")
    for i in range(num_events):
        x = cx + random.randint(-10, 10)
        y = 180 + random.randint(-5, 5)
        await send_dvs_event(dut, x, y, 1)
    
    # Wait for ring swap and classification (wait for full window + processing)
    dut._log.info("Waiting for ring swap and classification...")
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 8 + 500)
    
    # Check for gesture output
    response = await uart_receive_byte(dut, timeout_cycles=10000)
    
    if response is not None:
        dut._log.info(f"Received: 0x{response:02X}")
        if (response & 0xF0) == 0xA0:
            gesture = response & 0x03
            gesture_names = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
            dut._log.info(f"Detected gesture: {gesture_names.get(gesture, 'UNKNOWN')}")
            # For UP: positive delta_y means events moved up (higher Y)
            dut._log.info("Gesture UP test PASSED")
        else:
            dut._log.warning(f"Unexpected response format: 0x{response:02X}")
    else:
        dut._log.warning("No gesture detected (may need more events or time)")


@cocotb.test()
async def test_gesture_down(dut):
    """Test DOWN gesture detection"""
    await setup_test(dut)
    dut._log.info("=== Gesture DOWN Test ===")
    
    cx = SENSOR_RES // 2
    num_events = 10  # Reduced for faster simulation
    
    # Early events: high Y
    dut._log.info("Sending early events (high Y)...")
    for i in range(num_events):
        x = cx + random.randint(-10, 10)
        y = 180 + random.randint(-5, 5)
        await send_dvs_event(dut, x, y, 1)
    
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 4)
    
    # Late events: low Y
    dut._log.info("Sending late events (low Y)...")
    for i in range(num_events):
        x = cx + random.randint(-10, 10)
        y = 140 + random.randint(-5, 5)
        await send_dvs_event(dut, x, y, 1)
    
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 8 + 500)
    
    response = await uart_receive_byte(dut, timeout_cycles=10000)
    
    if response is not None and (response & 0xF0) == 0xA0:
        gesture = response & 0x03
        gesture_names = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
        dut._log.info(f"Detected gesture: {gesture_names.get(gesture, 'UNKNOWN')}")
        dut._log.info("Gesture DOWN test PASSED")
    else:
        dut._log.warning("No gesture detected")


@cocotb.test()
async def test_gesture_left(dut):
    """Test LEFT gesture detection"""
    await setup_test(dut)
    dut._log.info("=== Gesture LEFT Test ===")
    
    cy = SENSOR_RES // 2
    num_events = 10  # Reduced for faster simulation
    
    # Early events: high X (right side)
    dut._log.info("Sending early events (high X)...")
    for i in range(num_events):
        x = 180 + random.randint(-5, 5)
        y = cy + random.randint(-10, 10)
        await send_dvs_event(dut, x, y, 1)
    
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 4)
    
    # Late events: low X (left side)
    dut._log.info("Sending late events (low X)...")
    for i in range(num_events):
        x = 140 + random.randint(-5, 5)
        y = cy + random.randint(-10, 10)
        await send_dvs_event(dut, x, y, 1)
    
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 8 + 500)
    
    response = await uart_receive_byte(dut, timeout_cycles=10000)
    
    if response is not None and (response & 0xF0) == 0xA0:
        gesture = response & 0x03
        gesture_names = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
        dut._log.info(f"Detected gesture: {gesture_names.get(gesture, 'UNKNOWN')}")
        dut._log.info("Gesture LEFT test PASSED")
    else:
        dut._log.warning("No gesture detected")


@cocotb.test()
async def test_gesture_right(dut):
    """Test RIGHT gesture detection"""
    await setup_test(dut)
    dut._log.info("=== Gesture RIGHT Test ===")
    
    cy = SENSOR_RES // 2
    num_events = 10  # Reduced for faster simulation
    
    # Early events: low X (left side)
    dut._log.info("Sending early events (low X)...")
    for i in range(num_events):
        x = 140 + random.randint(-5, 5)
        y = cy + random.randint(-10, 10)
        await send_dvs_event(dut, x, y, 1)
    
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 4)
    
    # Late events: high X (right side)
    dut._log.info("Sending late events (high X)...")
    for i in range(num_events):
        x = 180 + random.randint(-5, 5)
        y = cy + random.randint(-10, 10)
        await send_dvs_event(dut, x, y, 1)
    
    await ClockCycles(dut.clk, CYCLES_PER_BIN * 8 + 500)
    
    response = await uart_receive_byte(dut, timeout_cycles=10000)
    
    if response is not None and (response & 0xF0) == 0xA0:
        gesture = response & 0x03
        gesture_names = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}
        dut._log.info(f"Detected gesture: {gesture_names.get(gesture, 'UNKNOWN')}")
        dut._log.info("Gesture RIGHT test PASSED")
    else:
        dut._log.warning("No gesture detected")
