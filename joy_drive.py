#!/usr/bin/env python3

import math
import os
import select
import sys
import termios
import threading
import tty

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray

VALID_KEYS = ['3', '4', '5', '6', '7', '8', ' ', 'h', '1', '2']

class AckermannDriveController(Node):
    def __init__(self):
        super().__init__('ackermann_drive_controller')

        # Publishers
        self.pub_angles = self.create_publisher(
            Float32MultiArray, 'target_angles', 10)
        self.pub_wheels = self.create_publisher(
            Float32MultiArray, '/wheel_speeds', 10)

        # Rover Dimensions
        self.L = 1.0  # Wheelbase
        self.W = 0.6  # Track Width

        # States
        self.mode = 'ackermann'  # 'ackermann' or 'crab'
        self.vx = 0.0
        self.center_steer = 0.0

        # Parameters
        self.lin_step = 0.1
        self.max_v = 1.0
        self.steer_step = 5.0  # 5-degree step

        # Key Repeat Filter Variables
        self.last_key = ''
        self.last_key_time = self.get_clock().now()

        # Actuator Variables
        self.actuator_cmd = 0.0
        self.last_actuator_time = self.get_clock().now()

        # Protects state shared between keyboard thread and publish timer
        self.lock = threading.Lock()
        self.running = True

        # Continuous Publishing Timer (20Hz)
        self.timer = self.create_timer(0.05, self.publish_state)

        # Start the keyboard thread 0.5s after spinning begins
        self.kb_start_timer = self.create_timer(0.5, self.start_keyboard)

        self.get_logger().info(
            "Ackermann Controller Started. [3] Fwd  [4] Left  [5] Rev  "
            "[6] Right  [7] Crab L  [8] Crab R  [h] Home  [SPACE] Stop  "
            "Actuator: [1]/[2]  Quit: Ctrl+C. Streaming at 20Hz..."
        )

    def start_keyboard(self):
        self.kb_start_timer.cancel()  # one-shot
        try:
            # Single-key input, no echo (Ctrl+C still raises SIGINT)
            self._tty_fd = sys.stdin.fileno()
            self._tty_attrs = termios.tcgetattr(self._tty_fd)
            tty.setcbreak(self._tty_fd)
            # Discard anything typed before we started listening
            termios.tcflush(self._tty_fd, termios.TCIFLUSH)
        except Exception as e:
            self._tty_attrs = None
            self.get_logger().error(f"Could not set up keyboard: {e}")
            return

        self.kb_thread = threading.Thread(
            target=self.keyboard_loop, daemon=True)
        self.kb_thread.start()

    def restore_terminal(self):
        attrs = getattr(self, '_tty_attrs', None)
        if attrs is not None:
            self._tty_attrs = None
            try:
                termios.tcsetattr(self._tty_fd, termios.TCSADRAIN, attrs)
            except Exception:
                pass

    def keyboard_loop(self):
        try:
            while rclpy.ok() and self.running:
                ready, _, _ = select.select([self._tty_fd], [], [], 0.1)
                if not ready:
                    continue
                data = os.read(self._tty_fd, 1)
                if not data:
                    break
                key = data.decode(errors='ignore').lower()
                if key in VALID_KEYS:
                    self.handle_key(key)
        except Exception as e:
            print(f"\nKeyboard error: {e}")
        finally:
            self.running = False
            self.restore_terminal()
            print("\nShutting down controller.")
            if rclpy.ok():
                rclpy.shutdown()

    def handle_key(self, key):
        with self.lock:
            now = self.get_clock().now()

            # Actuator Controls (1 = Push, 2 = Pull)
            if key == '1':
                self.actuator_cmd = 1.0
                self.last_actuator_time = now
                return
            elif key == '2':
                self.actuator_cmd = 2.0
                self.last_actuator_time = now
                return

            # High-Frequency Key Repeat Filter (speed keys only)
            if key in ['3', '5', '7', '8']:
                time_diff = (now - self.last_key_time).nanoseconds / 1e9

                if key == self.last_key and time_diff < 0.2:
                    self.last_key_time = now
                    return

                self.last_key = key
                self.last_key_time = now
            else:
                self.last_key = key
                self.last_key_time = now

            # Drive Controls
            if key == '3' and self.mode == 'ackermann':
                self.vx = min(self.max_v, self.vx + self.lin_step)

            elif key == '5' and self.mode == 'ackermann':
                self.vx = max(-self.max_v, self.vx - self.lin_step)

            # Steering Controls (Ackermann only)
            elif key == '6' and self.mode == 'ackermann':
                self.center_steer = max(
                    self.center_steer - self.steer_step, -45.0)

            elif key == '4' and self.mode == 'ackermann':
                self.center_steer = min(
                    self.center_steer + self.steer_step, 45.0)

            elif key in ('7', '8'):
                if self.mode != 'crab':
                    self.mode = 'crab'
                    self.vx = 0.0
                    self.center_steer = 0.0
                    self.get_logger().info("\n--- Mode: CRAB ---")
                if key == '7':
                    self.vx = min(self.max_v, self.vx + self.lin_step)
                else:
                    self.vx = max(-self.max_v, self.vx - self.lin_step)

            # Home: back to Ackermann
            elif key == 'h':
                self.mode = 'ackermann'
                self.vx = 0.0
                self.center_steer = 0.0

            # Emergency Stop
            elif key == ' ':
                self.vx = 0.0
                self.center_steer = 0.0
                self.last_key = ''

    def stop_and_publish(self):
        """Zero everything and send one final command (used on Ctrl+C)."""
        with self.lock:
            self.vx = 0.0
            self.center_steer = 0.0
            self.actuator_cmd = 0.0
        self.publish_state()

    def publish_state(self):
        with self.lock:
            # Actuator Auto-Stop Logic
            now = self.get_clock().now()
            if (now - self.last_actuator_time).nanoseconds / 1e9 > 0.2:
                self.actuator_cmd = 0.0

            msg_angles = Float32MultiArray()
            msg_wheels = Float32MultiArray()

            if self.mode == 'crab':
                angles = [90.0, 90.0, 90.0, 90.0]

                v_lf = -self.vx
                v_lr = -self.vx
                v_rf = self.vx
                v_rr = self.vx

            else:
                alpha = math.radians(self.center_steer)

                if abs(alpha) < 0.001:
                    theta_left = 0.0
                    theta_right = 0.0
                else:
                    R = self.L / math.tan(alpha)
                    theta_left = math.degrees(
                        math.atan(self.L / (R + self.W / 2.0)))
                    theta_right = math.degrees(
                        math.atan(self.L / (R - self.W / 2.0)))

                angles = [theta_left, 0.0, theta_right, 0.0]

            if self.mode == 'crab':
                pass  # wheel speeds already set above
            elif abs(self.center_steer) < 0.001:
                v_lf = -self.vx
                v_lr = -self.vx
                v_rf = self.vx
                v_rr = self.vx
            else:
                alpha = math.radians(abs(self.center_steer))
                R = self.L / math.tan(alpha)

                r_fixed_inner = R - self.W / 2.0
                r_fixed_outer = R + self.W / 2.0

                r_steered_inner = math.sqrt(
                    r_fixed_inner ** 2 + self.L ** 2)
                r_steered_outer = math.sqrt(
                    r_fixed_outer ** 2 + self.L ** 2)

                v_base = abs(self.vx)
                omega = v_base / r_steered_outer

                v_of = v_base
                v_if = omega * r_steered_inner
                v_ob = omega * r_fixed_outer
                v_ib = omega * r_fixed_inner

                if self.center_steer > 0:
                    mag_lf, mag_lr = v_of, v_ob
                    mag_rf, mag_rr = v_if, v_ib
                else:
                    mag_lf, mag_lr = v_if, v_ib
                    mag_rf, mag_rr = v_of, v_ob

                direction = 1.0 if self.vx >= 0 else -1.0

                v_lf = -mag_lf * direction
                v_lr = -mag_lr * direction
                v_rf = mag_rf * direction
                v_rr = mag_rr * direction

            # Publish
            msg_angles.data = [float(a) for a in angles]
            self.pub_angles.publish(msg_angles)

            msg_wheels.data = [
                float(v_lf),
                float(v_lr),
                float(v_rf),
                float(v_rr),
                float(self.actuator_cmd),
            ]
            self.pub_wheels.publish(msg_wheels)

            # Dashboard Feedback
            print(
                f"\rMode: {self.mode.upper():<9} | "
                f"Speed: {self.vx:5.1f} | "
                f"Input: {self.center_steer:5.1f}° | "
                f"W0(L): {angles[0]:5.1f}° | "
                f"W2(R): {angles[2]:5.1f}° | "
                f"Act: {int(self.actuator_cmd)}    ",
                end="",
                flush=True,
            )


def main(args=None):
    rclpy.init(args=args)
    node = AckermannDriveController()

    try:
        if rclpy.ok():
            rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Final zeroed command first, before anything shuts rclpy down
        try:
            if rclpy.ok():
                node.stop_and_publish()
        except Exception:
            pass
        node.running = False
        node.restore_terminal()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
