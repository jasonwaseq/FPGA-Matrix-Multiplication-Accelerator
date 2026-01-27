#!/usr/bin/env python3
"""
Ice40 Validation Script for DVS Gesture Accelerator with Temporal Voxel Binning

Sends mock DVS events via UART and verifies gesture classification.
Supports 320x320 DVS sensor resolution with 16x16 spatial compression.

UART Protocol:
  TX to FPGA: 5 bytes per event [X_HI, X_LO, Y_HI, Y_LO, POL]
              X_HI[0] = X[8], X_LO = X[7:0] (9-bit x coordinate)
              Y_HI[0] = Y[8], Y_LO = Y[7:0] (9-bit y coordinate)
              POL[0] = polarity (1=ON, 0=OFF)
  RX from FPGA: 1 byte gesture [0xA0 | gesture] where gesture = 0-3
                1 byte status  [0xB0 | bin] where bin = 0-7

Commands:
  Echo test: Send 0xFF, expect 0x55 back
  Status:    Send 0xFE, expect 0xBx back (x = current temporal bin)

Gestures:
  0 = UP, 1 = DOWN, 2 = LEFT, 3 = RIGHT

NOTE: Design uses internal power-on reset - no external button needed!
Just program the bitstream and it starts running immediately.
"""

import sys
import serial
import time
import argparse
import threading
import random
from typing import Optional, List, Tuple

# Configuration
SENSOR_RES = 320
GRID_SIZE = 16
BAUD_RATE_DEFAULT = 115200

GESTURES = {0: 'UP', 1: 'DOWN', 2: 'LEFT', 3: 'RIGHT'}


class GestureCapture:
    """Thread-safe gesture capture from serial port"""
    
    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.captured_gestures: List[int] = []
        self.running = False
        self.thread: Optional[threading.Thread] = None
    
    def start(self):
        """Start capturing in background"""
        self.captured_gestures = []
        self.running = True
        self.thread = threading.Thread(target=self._capture_loop, daemon=True)
        self.thread.start()
    
    def stop(self):
        """Stop capturing"""
        self.running = False
        if self.thread:
            self.thread.join(timeout=0.5)
    
    def _capture_loop(self):
        """Background capture loop"""
        while self.running:
            if self.ser.in_waiting > 0:
                try:
                    data = self.ser.read(1)
                    if data:
                        byte = data[0]
                        # Check for gesture marker (0xAx)
                        if (byte & 0xF0) == 0xA0:
                            gesture = byte & 0x03
                            self.captured_gestures.append(gesture)
                except Exception:
                    pass
            time.sleep(0.001)
    
    def get_last_gesture(self, timeout: float = 2.0) -> Optional[int]:
        """Wait for and return the most recent gesture"""
        start = time.time()
        initial_count = len(self.captured_gestures)
        
        while time.time() - start < timeout:
            if len(self.captured_gestures) > initial_count:
                return self.captured_gestures[-1]
            time.sleep(0.01)
        
        return self.captured_gestures[-1] if self.captured_gestures else None
    
    def clear(self):
        """Clear captured gestures"""
        self.captured_gestures = []


def send_event(ser: serial.Serial, x: int, y: int, polarity: int) -> None:
    """
    Send a single DVS event (5 bytes)
    
    Args:
        ser: Serial port
        x: X coordinate (0-319)
        y: Y coordinate (0-319)
        polarity: Event polarity (0 or 1)
    """
    x = max(0, min(319, x))
    y = max(0, min(319, y))
    
    x_hi = (x >> 8) & 0x01
    x_lo = x & 0xFF
    y_hi = (y >> 8) & 0x01
    y_lo = y & 0xFF
    pol = polarity & 0x01
    
    packet = bytes([x_hi, x_lo, y_hi, y_lo, pol])
    ser.write(packet)


def generate_gesture_events(direction: str, num_events: int = 300, 
                           spread: int = 60, noise: int = 10) -> List[Tuple[int, int, int]]:
    """
    Generate DVS events simulating a gesture movement.
    
    Events are generated with position changing over time to simulate movement:
    - RIGHT: X increases from center
    - LEFT:  X decreases from center
    - UP:    Y increases from center
    - DOWN:  Y decreases from center
    
    Args:
        direction: 'UP', 'DOWN', 'LEFT', or 'RIGHT'
        num_events: Total number of events to generate
        spread: Maximum movement distance in pixels
        noise: Random position noise amplitude
    
    Returns:
        List of (x, y, polarity) tuples
    """
    events = []
    cx, cy = SENSOR_RES // 2, SENSOR_RES // 2  # Center at 160, 160
    
    for i in range(num_events):
        progress = i / num_events  # 0 to 1
        
        # Add some noise for realism
        nx = random.randint(-noise, noise)
        ny = random.randint(-noise, noise)
        
        if direction == 'UP':
            x = cx + nx
            y = cy + int(progress * spread) + ny
        elif direction == 'DOWN':
            x = cx + nx
            y = cy - int(progress * spread) + ny
        elif direction == 'LEFT':
            x = cx - int(progress * spread) + nx
            y = cy + ny
        elif direction == 'RIGHT':
            x = cx + int(progress * spread) + nx
            y = cy + ny
        else:
            raise ValueError(f"Unknown direction: {direction}")
        
        # Clamp to valid range
        x = max(0, min(SENSOR_RES - 1, x))
        y = max(0, min(SENSOR_RES - 1, y))
        
        # Use ON events (polarity=1) for movement
        events.append((x, y, 1))
    
    return events


def test_uart_echo(ser: serial.Serial) -> bool:
    """Test basic UART connectivity by sending 0xFF and expecting 0x55 back"""
    print("  Testing UART echo...", end=' ', flush=True)
    
    ser.reset_input_buffer()
    ser.write(bytes([0xFF]))
    ser.flush()
    
    # Wait for response
    start = time.time()
    while time.time() - start < 0.5:
        if ser.in_waiting > 0:
            response = ser.read(1)
            if response and response[0] == 0x55:
                print("PASS (got 0x55)")
                return True
            else:
                print(f"FAIL (got 0x{response[0]:02X}, expected 0x55)")
                return False
        time.sleep(0.01)
    
    print("FAIL (no response - timeout)")
    return False


def test_status_query(ser: serial.Serial) -> Optional[int]:
    """Query current temporal bin status"""
    print("  Querying status...", end=' ', flush=True)
    
    ser.reset_input_buffer()
    ser.write(bytes([0xFE]))
    ser.flush()
    
    start = time.time()
    while time.time() - start < 0.5:
        if ser.in_waiting > 0:
            response = ser.read(1)
            if response:
                byte = response[0]
                if (byte & 0xF0) == 0xB0:
                    bin_num = byte & 0x07
                    print(f"OK (current bin: {bin_num})")
                    return bin_num
                else:
                    print(f"FAIL (unexpected: 0x{byte:02X})")
                    return None
        time.sleep(0.01)
    
    print("FAIL (no response)")
    return None


def test_gesture(ser: serial.Serial, direction: str, expected: int, 
                num_events: int = 300, verbose: bool = False) -> bool:
    """
    Send gesture events and verify classification result
    
    Args:
        ser: Serial port
        direction: Gesture direction name ('UP', 'DOWN', 'LEFT', 'RIGHT')
        expected: Expected gesture code (0-3)
        num_events: Number of events to send
        verbose: Print extra debug info
    
    Returns:
        True if correct gesture detected, False otherwise
    """
    print(f"  Testing {direction}...", end=' ', flush=True)
    
    # Clear any pending data
    ser.reset_input_buffer()
    
    # Start capturing responses in background
    capture = GestureCapture(ser)
    capture.start()
    
    # Generate and send events
    events = generate_gesture_events(direction, num_events)
    
    for i, (x, y, pol) in enumerate(events):
        send_event(ser, x, y, pol)
        # Small delay to avoid overwhelming UART buffer
        # At 115200 baud, 5 bytes takes ~434µs, so ~0.5ms delay is safe
        time.sleep(0.0005)
        
        if verbose and (i + 1) % 100 == 0:
            print(f"\n    Sent {i+1}/{num_events} events...", end=' ', flush=True)
    
    # Wait for classification result
    # Design needs ~800µs (2 windows) plus processing time
    result = capture.get_last_gesture(timeout=2.0)
    capture.stop()
    
    if result is None:
        print("FAIL (no gesture detected)")
        return False
    
    detected = GESTURES.get(result, f'UNKNOWN({result})')
    if result == expected:
        print(f"PASS -> {detected}")
        return True
    else:
        print(f"FAIL -> {detected} (expected {direction})")
        if verbose:
            print(f"    All captured: {[GESTURES.get(g, g) for g in capture.captured_gestures]}")
        return False


def test_coordinate_range(ser: serial.Serial) -> bool:
    """Test that all coordinate ranges are handled correctly"""
    print("  Testing coordinate range...", end=' ', flush=True)
    
    # Test corner and edge coordinates
    test_coords = [
        (0, 0),       # Top-left
        (319, 0),     # Top-right  
        (0, 319),     # Bottom-left
        (319, 319),   # Bottom-right
        (160, 160),   # Center
        (1, 1),       # Near origin
        (318, 318),   # Near max
    ]
    
    for x, y in test_coords:
        send_event(ser, x, y, 1)
        time.sleep(0.001)
    
    # Verify FPGA still responsive
    time.sleep(0.1)
    ser.reset_input_buffer()
    ser.write(bytes([0xFF]))
    ser.flush()
    
    time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.read(1)
        if response and response[0] == 0x55:
            print("PASS")
            return True
    
    print("FAIL (FPGA not responsive)")
    return False


def test_rapid_events(ser: serial.Serial, num_events: int = 200) -> bool:
    """Test handling of rapid event bursts"""
    print(f"  Testing rapid events ({num_events})...", end=' ', flush=True)
    
    # Send events as fast as possible
    for _ in range(num_events):
        x = random.randint(0, 319)
        y = random.randint(0, 319)
        send_event(ser, x, y, 1)
    
    ser.flush()
    time.sleep(0.2)
    
    # Verify still responsive
    ser.reset_input_buffer()
    ser.write(bytes([0xFF]))
    ser.flush()
    
    time.sleep(0.1)
    if ser.in_waiting > 0:
        response = ser.read(1)
        if response and response[0] == 0x55:
            print("PASS")
            return True
    
    print("FAIL")
    return False


def run_full_test(ser: serial.Serial, num_events: int = 300, verbose: bool = False) -> int:
    """
    Run the complete test suite
    
    Returns:
        Number of failed tests (0 = all passed)
    """
    print("\n" + "="*50)
    print(" DVS Gesture Accelerator Validation")
    print(" 320x320 sensor -> 16x16 grid -> gesture")
    print("="*50 + "\n")
    
    failures = 0
    
    # 1. Basic connectivity
    print("[1/7] UART Connectivity")
    if not test_uart_echo(ser):
        print("\n  UART echo test failed!")
        print("  Check: 1) FPGA is programmed with latest bitstream")
        print("         2) Correct serial port selected")
        print("         3) LED heartbeat is blinking (~3Hz)")
        print("         4) Baud rate is 115200\n")
        return 1  # Can't continue without UART
    
    # 2. Status query
    print("\n[2/7] Status Query")
    bin_num = test_status_query(ser)
    if bin_num is None:
        failures += 1
    
    # 3. Coordinate range
    print("\n[3/7] Coordinate Range")
    if not test_coordinate_range(ser):
        failures += 1
    
    # 4. Rapid events
    print("\n[4/7] Rapid Event Handling")
    if not test_rapid_events(ser):
        failures += 1
    
    # 5-8. Gesture tests
    print("\n[5/7] Gesture Classification")
    
    gesture_tests = [
        ('RIGHT', 3),
        ('LEFT', 2),
        ('UP', 0),
        ('DOWN', 1),
    ]
    
    gesture_passed = 0
    for direction, expected_code in gesture_tests:
        if test_gesture(ser, direction, expected_code, num_events, verbose):
            gesture_passed += 1
        else:
            failures += 1
        time.sleep(0.3)  # Gap between gesture tests
    
    # Summary
    print("\n" + "="*50)
    print(" RESULTS")
    print("="*50)
    
    total_tests = 7
    passed = total_tests - failures
    
    print(f"\n  Total:    {passed}/{total_tests} tests passed")
    print(f"  Gestures: {gesture_passed}/4 detected correctly")
    
    if failures == 0:
        print("\n  ✓ All tests PASSED!")
    else:
        print(f"\n  ✗ {failures} test(s) FAILED")
    
    print()
    return failures


def main():
    parser = argparse.ArgumentParser(
        description='Ice40 DVS Gesture Accelerator Validation',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  python validate_ice40.py /dev/ttyUSB0
  python validate_ice40.py COM3 -v
  python validate_ice40.py /dev/ttyACM0 -n 500

Protocol:
  Events are 5 bytes: [X_HI, X_LO, Y_HI, Y_LO, POL]
  Gestures return: 0xA0|gesture (UP=0, DOWN=1, LEFT=2, RIGHT=3)
  Echo test: 0xFF -> 0x55
  Status:    0xFE -> 0xB0|bin
''')
    parser.add_argument('port', help='Serial port (e.g., /dev/ttyUSB0, COM3)')
    parser.add_argument('-b', '--baud', type=int, default=BAUD_RATE_DEFAULT,
                       help=f'Baud rate (default: {BAUD_RATE_DEFAULT})')
    parser.add_argument('-n', '--num-events', type=int, default=300,
                       help='Events per gesture test (default: 300)')
    parser.add_argument('-v', '--verbose', action='store_true',
                       help='Verbose output')
    parser.add_argument('--reset', action='store_true',
                       help='Toggle DTR to reset FPGA before testing')
    parser.add_argument('--echo-only', action='store_true',
                       help='Only run echo test (quick connectivity check)')
    
    args = parser.parse_args()
    
    print(f"Connecting to {args.port} @ {args.baud} baud...")
    
    try:
        ser = serial.Serial(
            port=args.port,
            baudrate=args.baud,
            timeout=1,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        time.sleep(0.5)  # Wait for connection to stabilize
        
        if args.reset:
            print("Resetting FPGA via DTR...")
            ser.dtr = False
            time.sleep(0.1)
            ser.dtr = True
            time.sleep(0.5)
        
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        
        if args.echo_only:
            print("\n[Quick Test] UART Echo")
            success = test_uart_echo(ser)
            ser.close()
            return 0 if success else 1
        
        # Run full test suite
        failures = run_full_test(ser, args.num_events, args.verbose)
        
        ser.close()
        return 0 if failures == 0 else 1
        
    except serial.SerialException as e:
        print(f"Serial error: {e}")
        return 1
    except KeyboardInterrupt:
        print("\nAborted by user")
        return 1
    except Exception as e:
        print(f"Error: {e}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
