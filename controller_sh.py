#!/usr/bin/env python3
#frame convention: x = forward, y = left
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, String

L = 0.7777
W = 0.475

CMD_VEL_TIMEOUT_SEC = 0.3
JOY_CMD_TIMEOUT_SEC = 0.3  # joy_drive streams at 20 Hz while driving
PUBLISH_RATE_HZ = 20.0
MAX_WHEEL_SPEED = 1.0
MAX_LINEAR_ACCEL = 1.0    # m/s^2
MAX_ANGULAR_ACCEL = 2.0   # rad/s^2

WHEEL_POSITIONS = {
    "LF": (L / 2, W / 2),
    "LR": (-L / 2, W / 2),
    "RF": (L / 2, -W / 2),
    "RR": (-L / 2, -W / 2),
}

WHEEL_ORDER = ["RF", "RR", "LF", "LR"]

def compute_wheel_commands(vx, vy, wz):
    results = []
    for name in WHEEL_ORDER:
        x_i, y_i = WHEEL_POSITIONS[name]
        vx_i = vx - wz * y_i      # rigid-body velocity at a point: v_i = (v_center + w * r_i) in 2D, treating wz as a vector along +z,
        vy_i = vy + wz * x_i      # the cross product; w x r_i = wz*z_cap x (x_i, y_i, 0) = (-wz*y_i, wz*x_i)
        speed = math.hypot(vx_i, vy_i)
        angle_deg = math.degrees(math.atan2(vy_i, vx_i))
        results.append((angle_deg, speed))
    return results

def normalize_angle(angle_deg):
    angle_deg = angle_deg % 360.0
    if angle_deg > 180.0:
        angle_deg -= 360.0
    return angle_deg

def ramp_value(current, target, max_delta):
    if max_delta <= 0.0: return target
    if target > current: return min(target, current + max_delta)
    else: return max(target, current - max_delta)

def optimize_wheel(desired_angle_deg, desired_speed, last_angle_deg, speed_deadband=1e-3, boundary_tol=1e-6):
    if desired_speed < speed_deadband:
        return last_angle_deg, 0.0
 
    angle = normalize_angle(desired_angle_deg)
    if angle > 90.0:
        angle -= 180.0
        desired_speed = -desired_speed
    elif angle < -90.0:
        angle += 180.0
        desired_speed = -desired_speed

    # crab fix
    if abs(abs(angle) - 90.0) < boundary_tol:
        alt_angle = -angle
        current_dist = abs(normalize_angle(angle - last_angle_deg))
        alt_dist = abs(normalize_angle(alt_angle - last_angle_deg))
        if alt_dist < current_dist:
            angle = alt_angle
            desired_speed = -desired_speed
 
    return angle, desired_speed

class SwerveKinematicsNode(Node):
    def __init__(self):
        super().__init__('swerve_kinematics_node')

        self.declare_parameter('max_wheel_speed', MAX_WHEEL_SPEED)
        self.declare_parameter('cmd_vel_timeout', CMD_VEL_TIMEOUT_SEC)
        self.declare_parameter('max_linear_accel', MAX_LINEAR_ACCEL)
        self.declare_parameter('max_angular_accel', MAX_ANGULAR_ACCEL)
        self.declare_parameter('joy_cmd_timeout', JOY_CMD_TIMEOUT_SEC)

        self.max_wheel_speed = self.get_parameter('max_wheel_speed').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        self.max_linear_accel = self.get_parameter('max_linear_accel').value
        self.max_angular_accel = self.get_parameter('max_angular_accel').value
        self.joy_timeout = self.get_parameter('joy_cmd_timeout').value

        self.vx = 0.0
        self.vy = 0.0
        self.wz = 0.0
        self.last_cmd_time = self.get_clock().now()

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_wz = 0.0
        self.last_update_time = self.get_clock().now()

        self.last_angles = {name: 0.0 for name in WHEEL_ORDER}

        self.cmd_moving = False
        self.joy_angles = [0.0] * 4
        self.joy_speeds = [0.0] * 4
        self.joy_seen = False
        self.joy_seq = None
        self.last_joy_time = self.get_clock().now()
        self.source = 'teleop'
        self._logged_source = self.source
        self._src_key = None
        self._src_time = self.get_clock().now()

        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_joy = self.create_subscription(Float32MultiArray, '/joy_drive_cmd', self.joy_callback, 10)

        self.pub_angles = self.create_publisher(Float32MultiArray, 'target_angles', 10)
        self.pub_wheels = self.create_publisher(Float32MultiArray, '/wheel_speeds', 10)
        self.pub_source = self.create_publisher(String, '/drive_source', 10)

        self.timer = self.create_timer(1.0 / PUBLISH_RATE_HZ, self.publish_wheel_commands)

        self.get_logger().info('swerve_kinematics_node started, publishing at 'f'{PUBLISH_RATE_HZ} Hz (cmd_vel timeout {self.cmd_vel_timeout}s)')

    def cmd_vel_callback(self, msg):
        self.vx = msg.linear.x
        self.vy = msg.linear.y
        self.wz = msg.angular.z
        self.cmd_moving = any(abs(v) > 1e-6 for v in (self.vx, self.vy, self.wz))
        self.last_cmd_time = self.get_clock().now()
        if self.cmd_moving:
            self.source = 'teleop'

    def joy_callback(self, msg):
        if len(msg.data) < 9:
            return
        self.joy_angles = [float(a) for a in msg.data[:4]]
        self.joy_speeds = [float(s) for s in msg.data[4:8]]
        self.joy_seen = True
        self.last_joy_time = self.get_clock().now()
        seq = int(round(msg.data[8]))
        if seq != self.joy_seq:
            self.joy_seq = seq
            self.source = 'joy'

    def publish_source(self, now):
        key = f"{self.source} {-1 if self.joy_seq is None else self.joy_seq}"
        if key != self._src_key or (now - self._src_time).nanoseconds / 1e9 > 1.0:
            msg = String()
            msg.data = key
            self.pub_source.publish(msg)
            self._src_key = key
            self._src_time = now

    def publish_wheel_commands(self):
        now = self.get_clock().now()

        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now
        if dt <= 0.0: dt = 1.0 / PUBLISH_RATE_HZ

        age = (now - self.last_cmd_time).nanoseconds / 1e9
        joy_age = (now - self.last_joy_time).nanoseconds / 1e9
        joy_fresh = self.joy_seen and joy_age <= self.joy_timeout

        # Single publisher: self.source was set by whichever program's key was hit last.
        if self.source != self._logged_source:
            self.get_logger().info(f'drive source: {self._logged_source} -> {self.source}')
            self._logged_source = self.source
        self.publish_source(now)

        if self.source == 'joy':
            self.current_vx = self.current_vy = self.current_wz = 0.0
            if joy_fresh:
                angles_deg = list(self.joy_angles)
                speeds_norm = [max(-1.0, min(1.0, s)) for s in self.joy_speeds]
                for name, angle in zip(WHEEL_ORDER, angles_deg):
                    self.last_angles[name] = angle
            else:
                # joy is idle (homed/stopped, stream ended): hold angles, zero speed
                angles_deg = [self.last_angles[name] for name in WHEEL_ORDER]
                speeds_norm = [0.0] * 4
        else:
            if age > self.cmd_vel_timeout: target_vx, target_vy, target_wz = 0.0, 0.0, 0.0
            else: target_vx, target_vy, target_wz = self.vx, self.vy, self.wz

            max_lin_delta = self.max_linear_accel * dt
            max_ang_delta = self.max_angular_accel * dt
            self.current_vx = ramp_value(self.current_vx, target_vx, max_lin_delta)
            self.current_vy = ramp_value(self.current_vy, target_vy, max_lin_delta)
            self.current_wz = ramp_value(self.current_wz, target_wz, max_ang_delta)

            raw_commands = compute_wheel_commands(self.current_vx, self.current_vy, self.current_wz)

            angles_deg = []
            speeds_norm = []
            for name, (raw_angle, raw_speed) in zip(WHEEL_ORDER, raw_commands):
                angle, speed = optimize_wheel(raw_angle, raw_speed, self.last_angles[name])
                self.last_angles[name] = angle

                angles_deg.append(angle)
                normalized = speed / self.max_wheel_speed
                speeds_norm.append(max(-1.0, min(1.0, normalized)))

            for i, name in enumerate(WHEEL_ORDER):
                if name in ('RF', 'RR'): speeds_norm[i] = -speeds_norm[i]

        angles_msg = Float32MultiArray()
        angles_msg.data = [float(a) for a in angles_deg]
        self.pub_angles.publish(angles_msg)

        wheels_msg = Float32MultiArray()
        wheels_msg.data = [float(s) for s in speeds_norm] + [0.0]
        self.pub_wheels.publish(wheels_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SwerveKinematicsNode()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
