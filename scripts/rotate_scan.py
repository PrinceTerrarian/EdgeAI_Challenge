#!/usr/bin/env python3

import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def get_yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class RotateScan(Node):

    def __init__(self, target_deg, max_speed):
        super().__init__('rotate_scan')

        self.target = math.radians(abs(target_deg))
        self.max_speed = max_speed

        self.previous_yaw = None
        self.total_angle = 0.0
        self.last_odom = None

        self.publisher = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_sensor_data
        )

    def odom_callback(self, msg):
        yaw = get_yaw(msg.pose.pose.orientation)

        if self.previous_yaw is not None:
            delta = math.atan2(
                math.sin(yaw - self.previous_yaw),
                math.cos(yaw - self.previous_yaw)
            )
            self.total_angle += delta

        self.previous_yaw = yaw
        self.last_odom = time.monotonic()

    def send_velocity(self, speed):
        msg = Twist()
        msg.angular.z = speed
        self.publisher.publish(msg)

    def stop(self):
        for _ in range(10):
            self.send_velocity(0.0)
            time.sleep(0.05)

    def run_rotation(self):
        print("Waiting for odometry...")

        start_wait = time.monotonic()

        while rclpy.ok() and self.previous_yaw is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - start_wait > 15:
                raise RuntimeError("No odometry received")

        print("Starting rotation")

        start_time = time.monotonic()
        last_time = start_time
        current_speed = 0.0

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            now = time.monotonic()

            if now - start_time > 600:
                raise RuntimeError("Rotation timeout")

            if now - self.last_odom > 5:
                raise RuntimeError("Odometry data timeout")

            remaining = self.target - self.total_angle

            if remaining <= math.radians(1.0):
                print("\nRotation completed")
                break

            # Slow down near target
            desired_speed = min(
                self.max_speed,
                math.sqrt(max(0.0, 0.24 * remaining))
            )

            # Acceleration limit
            dt = min(now - last_time, 0.2)
            step = 0.12 * dt

            if current_speed < desired_speed:
                current_speed = min(
                    current_speed + step, desired_speed
                )
            else:
                current_speed = max(
                    current_speed - step, desired_speed
                )

            self.send_velocity(current_speed)

            print(
                f"\rRotation: "
                f"{math.degrees(self.total_angle):.1f}"
                f" / {math.degrees(self.target):.0f} deg",
                end="", flush=True
            )

            last_time = now
            time.sleep(0.05)

    def finish(self):
        self.stop()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '--angle', type=float, default=360.0
    )
    parser.add_argument(
        '--speed', type=float, default=0.15
    )
    args = parser.parse_args()

    if not (0 < args.angle <= 3600):
        parser.error("angle must be between 0 and 3600 degrees")
    if not (0 < args.speed <= 1.0):
        parser.error("speed must be between 0 and 1 rad/s")

    rclpy.init()

    node = RotateScan(args.angle, args.speed)

    try:
        node.run_rotation()
    except KeyboardInterrupt:
        print("\nInterrupted")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        node.finish()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
