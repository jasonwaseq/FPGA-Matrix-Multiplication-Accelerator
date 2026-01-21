"""
Cocotb testbench for simplified DVS gesture detection.
Tests directional gesture recognition: UP, DOWN, LEFT, RIGHT.
"""

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import RisingEdge, Timer
import random


async def send_event(dut, x, y, polarity, timestamp):
    """Send a single DVS event."""
    dut.dvs_event_valid_i.value = 1
    dut.dvs_x_i.value = x
    dut.dvs_y_i.value = y
    dut.dvs_polarity_i.value = polarity
    dut.dvs_timestamp_i.value = timestamp
    await RisingEdge(dut.clk_i)
    while dut.dvs_ready_o.value == 0:
        await RisingEdge(dut.clk_i)
    dut.dvs_event_valid_i.value = 0


async def wait_for_gesture(dut, max_cycles=5000):
    """Wait for a gesture output."""
    for _ in range(max_cycles):
        await RisingEdge(dut.clk_i)
        if dut.gesture_valid_o.value == 1:
            gesture = int(dut.gesture_idx_o.value)
            confidence = int(dut.confidence_score_o.value)
            return gesture, confidence
    return None, None


@cocotb.test()
async def test_dvs_gesture_up(dut):
    """Test UP gesture detection."""
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset_i.value = 1
    dut.dvs_event_valid_i.value = 0
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    dut.reset_i.value = 0
    await RisingEdge(dut.clk_i)

    # Send UP gesture: more activity in top half with increasing timestamps
    dut._log.info("Sending UP gesture events...")
    timestamp = 0
    for _ in range(20):
        for y in range(4):  # Top half (0-3)
            for x in range(8):
                await send_event(dut, x, y, 1, timestamp)
                timestamp += 1

    # Send event with timestamp >= 1000 to trigger decay and classification
    dut._log.info("Triggering decay with timestamp 1000...")
    await send_event(dut, 0, 0, 1, 1000)

    # Wait for classification (should happen shortly after decay)
    gesture, confidence = await wait_for_gesture(dut, max_cycles=500)
    dut._log.info(f"Detected gesture: {gesture}, confidence: {confidence}")
    assert gesture is not None, "No gesture detected"
    assert gesture == 0, f"Expected UP (0), got {gesture}"


@cocotb.test()
async def test_dvs_gesture_down(dut):
    """Test DOWN gesture detection."""
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset_i.value = 1
    dut.dvs_event_valid_i.value = 0
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    dut.reset_i.value = 0
    await RisingEdge(dut.clk_i)

    # Send DOWN gesture: more activity in bottom half
    dut._log.info("Sending DOWN gesture events...")
    timestamp = 0
    for _ in range(20):
        for y in range(4, 8):  # Bottom half (4-7)
            for x in range(8):
                await send_event(dut, x, y, 1, timestamp)
                timestamp += 1

    # Trigger decay and classification
    dut._log.info("Triggering decay with timestamp 1000...")
    await send_event(dut, 0, 0, 1, 1000)

    # Wait for classification
    gesture, confidence = await wait_for_gesture(dut, max_cycles=500)
    dut._log.info(f"Detected gesture: {gesture}, confidence: {confidence}")
    assert gesture is not None, "No gesture detected"
    assert gesture == 1, f"Expected DOWN (1), got {gesture}"


@cocotb.test()
async def test_dvs_gesture_left(dut):
    """Test LEFT gesture detection."""
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset_i.value = 1
    dut.dvs_event_valid_i.value = 0
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    dut.reset_i.value = 0
    await RisingEdge(dut.clk_i)

    # Send LEFT gesture: more activity in left half
    dut._log.info("Sending LEFT gesture events...")
    timestamp = 0
    for _ in range(20):
        for y in range(8):
            for x in range(4):  # Left half (0-3)
                await send_event(dut, x, y, 1, timestamp)
                timestamp += 1

    # Trigger decay and classification
    dut._log.info("Triggering decay with timestamp 1000...")
    await send_event(dut, 0, 0, 1, 1000)

    # Wait for classification
    gesture, confidence = await wait_for_gesture(dut, max_cycles=500)
    dut._log.info(f"Detected gesture: {gesture}, confidence: {confidence}")
    assert gesture is not None, "No gesture detected"
    assert gesture == 2, f"Expected LEFT (2), got {gesture}"


@cocotb.test()
async def test_dvs_gesture_right(dut):
    """Test RIGHT gesture detection."""
    clock = Clock(dut.clk_i, 10, units="ns")
    cocotb.start_soon(clock.start())

    dut.reset_i.value = 1
    dut.dvs_event_valid_i.value = 0
    await RisingEdge(dut.clk_i)
    await RisingEdge(dut.clk_i)
    dut.reset_i.value = 0
    await RisingEdge(dut.clk_i)

    # Send RIGHT gesture: more activity in right half
    dut._log.info("Sending RIGHT gesture events...")
    timestamp = 0
    for _ in range(20):
        for y in range(8):
            for x in range(4, 8):  # Right half (4-7)
                await send_event(dut, x, y, 1, timestamp)
                timestamp += 1

    # Trigger decay and classification
    dut._log.info("Triggering decay with timestamp 1000...")
    await send_event(dut, 0, 0, 1, 1000)

    # Wait for classification
    gesture, confidence = await wait_for_gesture(dut, max_cycles=500)
    dut._log.info(f"Detected gesture: {gesture}, confidence: {confidence}")
    assert gesture is not None, "No gesture detected"
    assert gesture == 3, f"Expected RIGHT (3), got {gesture}"

