#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Keyboard Joystick Simulator for Dragon Copilot

This script simulates a joystick using keyboard inputs, allowing control
of the dragon_copilot without physical joystick hardware.

Controls:
  W/S       - Left stick up/down (Pitch velocity)
  A/D       - Left stick left/right (Yaw velocity)
  I/K       - Right stick up/down (Z velocity / altitude)
  J/L       - Right stick left/right (Y velocity / lateral)
  U/O       - Triggers R2/L2 (Forward/Backward X velocity)
  
  R       - START button (Motor arming)
  E       - STOP button (Force landing/halt)
  T       - CROSS_LEFT + ACTION_CIRCLE (Takeoff)
  G       - CROSS_RIGHT + ACTION_SQUARE (Landing)
  
  ESC     - Exit program

Author: GitHub Copilot
Date: 2025-12-06
"""

import rospy
from sensor_msgs.msg import Joy
import sys
import select
import termios
import tty
import threading
import time


class KeyboardJoystickSimulator:
    def __init__(self):
        """Initialize the keyboard joystick simulator."""
        rospy.init_node('keyboard_joystick_simulator', anonymous=True)

        # Publisher for joystick messages
        self.joy_pub = rospy.Publisher('joy', Joy, queue_size=10)

        # Initialize joystick message (PS4 format: 14 axes, 14 buttons)
        self.joy_msg = Joy()
        self.joy_msg.header.frame_id = "keyboard_joy"

        # PS4 joystick format
        # Axes: 14 axes (PS4 standard)
        self.joy_msg.axes = [0.0] * 14
        # Buttons: 14 buttons
        self.joy_msg.buttons = [0] * 14

        # Axis indices (PS4 format - matching joy_parser.h)
        self.AXIS_LEFT_X = 0                  # Left stick horizontal (yaw)
        self.AXIS_LEFT_Y = 1                  # Left stick vertical (pitch)
        self.AXIS_RIGHT_X = 2                 # Right stick horizontal (lateral Y)
        self.AXIS_L2 = 3                      # L2 trigger (backward)
        self.AXIS_R2 = 4                      # R2 trigger (forward)
        self.AXIS_RIGHT_Y = 5                 # Right stick vertical (altitude Z)
        self.AXIS_ACCEL_LEFT = 6              # Accelerometer (unused)
        self.AXIS_ACCEL_FORWARD = 7           # Accelerometer (unused)
        self.AXIS_ACCEL_UP = 8                # Accelerometer (unused)
        self.AXIS_DPAD_X = 9                  # D-pad horizontal (Cross Left/Right)
        self.AXIS_DPAD_Y = 10                 # D-pad vertical (Cross Up/Down)
        self.AXIS_GYRO_ROLL = 11              # Gyro (unused)
        self.AXIS_GYRO_YAW = 12               # Gyro (unused)
        self.AXIS_GYRO_PITCH = 13             # Gyro (unused)

        # Button indices (PS4 format - matching joy_parser.h)
        self.BUTTON_SQUARE = 0       # Square button
        self.BUTTON_CROSS = 1        # Cross button (X)
        self.BUTTON_CIRCLE = 2       # Circle button
        self.BUTTON_TRIANGLE = 3     # Triangle button
        self.BUTTON_L1 = 4           # L1 button
        self.BUTTON_R1 = 5           # R1 button
        self.BUTTON_L2 = 6           # L2 button (also has axis)
        self.BUTTON_R2 = 7           # R2 button (also has axis)
        self.BUTTON_SHARE = 8        # Share button (STOP)
        self.BUTTON_OPTIONS = 9      # Options button (START)
        self.BUTTON_L3 = 10          # Left stick button
        self.BUTTON_R3 = 11          # Right stick button
        self.BUTTON_PAIRING = 12     # Pairing button
        self.BUTTON_TOUCHPAD = 13    # Touchpad button

        # Initialize triggers to neutral position (+1.0) and unused axes to 0
        self.joy_msg.axes[self.AXIS_L2] = 1.0
        self.joy_msg.axes[self.AXIS_R2] = 1.0

        # Control parameters
        self.publish_rate = 20.0   # Publishing rate in Hz

        # Key press states - track which keys are currently pressed
        self.key_states = {}

        # Key timeout - if a key is not pressed again within this time, consider it released
        self.key_timeout = 0.15  # 150ms timeout
        self.key_timestamps = {}  # Track last press time for each key

        # Store old terminal settings
        self.old_settings = None

        # Running flag
        self.running = True

        # Button hold counter - buttons will be held for multiple publish cycles
        self.button_hold_cycles = 3  # Hold buttons for 3 cycles (~150ms at 20Hz)
        self.button_hold_counters = {}  # Track how many cycles each button has been held
        self.dpad_hold_counters = {}  # Separate tracking for D-pad axes

        # Display instructions
        self.print_instructions()

    def print_instructions(self):
        """Print control instructions."""
        print("\n" + "="*60)
        print("  Keyboard Joystick Simulator for Dragon Copilot")
        print("="*60)
        print("\nFlight Controls (Hold to activate):")
        print("  W/S       - Pitch forward/backward")
        print("  A/D       - Yaw left/right")
        print("  I/K       - Altitude up/down")
        print("  J/L       - Lateral left/right")
        print("  U/O       - Forward/Backward (R2/L2 triggers)")
        print("\nCommand Buttons:")
        print("  R       - START (Motor arming)")
        print("  E       - STOP (Force landing/halt)")
        print("  T       - TAKEOFF (Cross Left + Circle)")
        print("  G       - LANDING (Cross Right + Square)")
        print("\nUtility:")
        print("  ESC     - Exit program")
        print("="*60)
        print("\nJoystick format: PS4 (14 axes, 14 buttons)")
        print("Publishing joystick commands at %.1f Hz..." % self.publish_rate)
        print("Current state: [Waiting for input]\n")

    def reset_buttons(self):
        """Reset all buttons to unpressed state."""
        self.joy_msg.buttons = [0] * 14
        # Also reset D-pad axes
        self.joy_msg.axes[self.AXIS_DPAD_X] = 0.0
        self.joy_msg.axes[self.AXIS_DPAD_Y] = 0.0

    def cleanup_expired_keys(self):
        """Remove keys that haven't been pressed within the timeout period."""
        current_time = time.time()
        expired_keys = []

        for key, timestamp in self.key_timestamps.items():
            if current_time - timestamp > self.key_timeout:
                expired_keys.append(key)

        for key in expired_keys:
            if key in self.key_states:
                del self.key_states[key]
            del self.key_timestamps[key]

    def update_axes_from_keys(self):
        """Update joystick axes based on currently pressed keys."""
        # First, clean up any expired keys
        self.cleanup_expired_keys()

        # Left stick Y (pitch): W/S
        if 'w' in self.key_states and self.key_states['w']:
            self.joy_msg.axes[self.AXIS_LEFT_Y] = 1.0
        elif 's' in self.key_states and self.key_states['s']:
            self.joy_msg.axes[self.AXIS_LEFT_Y] = -1.0
        else:
            self.joy_msg.axes[self.AXIS_LEFT_Y] = 0.0

        # Left stick X (yaw): A/D
        if 'a' in self.key_states and self.key_states['a']:
            self.joy_msg.axes[self.AXIS_LEFT_X] = 1.0
        elif 'd' in self.key_states and self.key_states['d']:
            self.joy_msg.axes[self.AXIS_LEFT_X] = -1.0
        else:
            self.joy_msg.axes[self.AXIS_LEFT_X] = 0.0

        # Right stick Y (altitude Z): I/K
        if 'i' in self.key_states and self.key_states['i']:
            self.joy_msg.axes[self.AXIS_RIGHT_Y] = 1.0
        elif 'k' in self.key_states and self.key_states['k']:
            self.joy_msg.axes[self.AXIS_RIGHT_Y] = -1.0
        else:
            self.joy_msg.axes[self.AXIS_RIGHT_Y] = 0.0

        # Right stick X (lateral Y): J/L
        if 'j' in self.key_states and self.key_states['j']:
            self.joy_msg.axes[self.AXIS_RIGHT_X] = 1.0
        elif 'l' in self.key_states and self.key_states['l']:
            self.joy_msg.axes[self.AXIS_RIGHT_X] = -1.0
        else:
            self.joy_msg.axes[self.AXIS_RIGHT_X] = 0.0

        # Triggers: U/O keys for R2/L2 (binary: on/off)
        if 'u' in self.key_states and self.key_states['u']:
            self.joy_msg.axes[self.AXIS_R2] = -1.0  # Pressed
        else:
            self.joy_msg.axes[self.AXIS_R2] = 1.0   # Neutral

        if 'o' in self.key_states and self.key_states['o']:
            self.joy_msg.axes[self.AXIS_L2] = -1.0  # Pressed
        else:
            self.joy_msg.axes[self.AXIS_L2] = 1.0   # Neutral

    def set_button(self, button_index):
        """Set a button to be pressed for multiple cycles."""
        self.button_hold_counters[button_index] = self.button_hold_cycles
        self.joy_msg.buttons[button_index] = 1

    def update_button_states(self):
        """Update button states based on hold counters."""
        for button_index in list(self.button_hold_counters.keys()):
            self.button_hold_counters[button_index] -= 1
            if self.button_hold_counters[button_index] <= 0:
                del self.button_hold_counters[button_index]
                self.joy_msg.buttons[button_index] = 0
            else:
                self.joy_msg.buttons[button_index] = 1

        # Update D-pad hold counters separately
        for dpad_key in list(self.dpad_hold_counters.keys()):
            self.dpad_hold_counters[dpad_key] -= 1
            if self.dpad_hold_counters[dpad_key] <= 0:
                del self.dpad_hold_counters[dpad_key]

    def process_key(self, key):
        """Process keyboard input and update joystick state."""
        current_time = time.time()

        # === Axis controls (binary: on/off) ===
        # Left stick Y (pitch): W/S
        if key == 'w':
            self.key_states['w'] = True
            self.key_timestamps['w'] = current_time
        elif key == 's':
            self.key_states['s'] = True
            self.key_timestamps['s'] = current_time

        # Left stick X (yaw): A/D
        elif key == 'a':
            self.key_states['a'] = True
            self.key_timestamps['a'] = current_time
        elif key == 'd':
            self.key_states['d'] = True
            self.key_timestamps['d'] = current_time

        # Right stick Y (altitude Z): I/K
        elif key == 'i':
            self.key_states['i'] = True
            self.key_timestamps['i'] = current_time
        elif key == 'k':
            self.key_states['k'] = True
            self.key_timestamps['k'] = current_time

        # Right stick X (lateral Y): J/L
        elif key == 'j':
            self.key_states['j'] = True
            self.key_timestamps['j'] = current_time
        elif key == 'l':
            self.key_states['l'] = True
            self.key_timestamps['l'] = current_time

        # Triggers: U/O keys for R2/L2 (binary: on/off)
        elif key == 'u':
            self.key_states['u'] = True
            self.key_timestamps['u'] = current_time
        elif key == 'o':
            self.key_states['o'] = True
            self.key_timestamps['o'] = current_time

        # ESC - Exit
        elif key == '\x1b':  # ESC key
            print("\nExiting...")
            self.running = False
            return False

        # === Button controls ===
        elif key == 'r':
            self.set_button(self.BUTTON_OPTIONS)  # START
            print("START button pressed (Motor arming)")

        elif key == 'e':
            self.set_button(self.BUTTON_SHARE)  # STOP
            print("STOP button pressed (Force landing/halt)")

        elif key == 't':
            # TAKEOFF: Cross Left + Circle
            self.joy_msg.axes[self.AXIS_DPAD_X] = 1.0  # Cross Left
            self.dpad_hold_counters['dpad_x_left'] = self.button_hold_cycles
            self.set_button(self.BUTTON_CIRCLE)
            print("TAKEOFF command (Cross Left + Circle)")

        elif key == 'g':
            # LANDING: Cross Right + Square
            self.joy_msg.axes[self.AXIS_DPAD_X] = -1.0  # Cross Right
            self.dpad_hold_counters['dpad_x_right'] = self.button_hold_cycles
            self.set_button(self.BUTTON_SQUARE)
            print("LANDING command (Cross Right + Square)")

        return True

    def get_key(self):
        """Get a single key press from the terminal."""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

    def publish_loop(self):
        """Main publishing loop."""
        rate = rospy.Rate(self.publish_rate)

        while not rospy.is_shutdown() and self.running:
            # Update axes based on currently pressed keys
            self.update_axes_from_keys()

            # Update button states (decrease hold counters)
            self.update_button_states()

            # Update D-pad axes based on hold counters
            if 'dpad_x_left' in self.dpad_hold_counters:
                self.joy_msg.axes[self.AXIS_DPAD_X] = 1.0
            elif 'dpad_x_right' in self.dpad_hold_counters:
                self.joy_msg.axes[self.AXIS_DPAD_X] = -1.0
            else:
                self.joy_msg.axes[self.AXIS_DPAD_X] = 0.0

            # Update timestamp
            self.joy_msg.header.stamp = rospy.Time.now()

            # Publish joystick message
            self.joy_pub.publish(self.joy_msg)

            # Sleep to maintain rate
            rate.sleep()

    def keyboard_loop(self):
        """Keyboard input loop."""
        try:
            while self.running and not rospy.is_shutdown():
                key = self.get_key()
                if key:
                    if not self.process_key(key.lower()):
                        break
                rospy.sleep(0.001)  # Small sleep to prevent CPU spinning
        except Exception as e:
            print("\nError in keyboard loop: %s" % str(e))
        finally:
            self.cleanup()

    def setup_terminal(self):
        """Setup terminal for raw input."""
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    def cleanup(self):
        """Restore terminal settings."""
        if self.old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        print("\nTerminal settings restored.")

    def run(self):
        """Run the keyboard joystick simulator."""
        try:
            # Setup terminal for raw input
            self.setup_terminal()

            # Start publishing thread
            publish_thread = threading.Thread(target=self.publish_loop)
            publish_thread.daemon = True
            publish_thread.start()

            # Start keyboard input loop (blocking)
            self.keyboard_loop()

        except KeyboardInterrupt:
            print("\n\nInterrupted by user (Ctrl+C)")
        except Exception as e:
            print("\nError: %s" % str(e))
        finally:
            self.running = False
            self.cleanup()
            print("Keyboard joystick simulator stopped.")


def main():
    try:
        simulator = KeyboardJoystickSimulator()
        simulator.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
