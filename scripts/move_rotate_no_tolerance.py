#!/usr/bin/env python3

import math
import time
import argparse

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def get_yaw(q):
    return math.atan2(
        2 * (q.w * q.z + q.x * q.y),
        1 - 2 * (q.y * q.y + q.z * q.z)
    )


class MoveRotate(Node):

    def __init__(self, distance, linear_speed, angular_speed):
        super().__init__("move_rotate_no_tolerance")

        self.distance = distance
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

        self.x = None
        self.y = None
        self.yaw = None

        self.last_odom = None
        self.last_scan = None
        self.front_distance = None

        self.publisher = self.create_publisher(
            Twist, "/cmd_vel", 10
        )

        self.create_subscription(
            Odometry, "/odom",
            self.odom_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            LaserScan, "/scan",
            self.scan_callback,
            qos_profile_sensor_data
        )

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = get_yaw(msg.pose.pose.orientation)
        self.last_odom = time.monotonic()

    def scan_callback(self, msg):
        values = []

        for i, d in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment

            if abs(normalize(angle)) <= math.radians(20):
                if math.isfinite(d):
                    if msg.range_min <= d <= msg.range_max:
                        values.append(d)

        self.front_distance = min(values) if values else None
        self.last_scan = time.monotonic()

    def command(self, linear=0.0, angular=0.0):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher.publish(msg)

    def stop(self):
        for _ in range(10):
            self.command()
            time.sleep(0.05)

    def update(self):
        rclpy.spin_once(self, timeout_sec=0.05)
        now = time.monotonic()

        if self.last_odom is None or now - self.last_odom > 5:
            raise RuntimeError("Odometry unavailable")

        if self.last_scan is None or now - self.last_scan > 5:
            raise RuntimeError("Laser scan unavailable")

    def wait_for_data(self):
        print("Waiting for /odom and /scan...")
        start = time.monotonic()

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            if self.last_odom is not None and self.last_scan is not None:
                print("Sensors ready")
                return

            if time.monotonic() - start > 20:
                raise RuntimeError("Sensor startup timeout")

    def rotate(self, angle_deg):
        target = math.radians(abs(angle_deg))
        direction = 1 if angle_deg >= 0 else -1

        previous_yaw = self.yaw
        total = 0.0
        speed = 0.0
        start = time.monotonic()

        print(f"\nRotating {angle_deg} degrees")

        while rclpy.ok():
            self.update()

            delta = normalize(self.yaw - previous_yaw)
            total += direction * delta
            previous_yaw = self.yaw

            remaining = target - total

            # No early stopping tolerance
            if remaining <= 0.0:
                break

            if time.monotonic() - start > 600:
                raise RuntimeError("Rotation timeout")

            # Slow down near target
            desired = min(
                self.angular_speed,
                max(0.02, math.sqrt(0.12 * remaining))
            )

            if speed < desired:
                speed = min(speed + 0.006, desired)
            else:
                speed = max(speed - 0.006, desired)

            self.command(angular=direction * speed)

            print(
                f"\rRotated: {math.degrees(total):.2f}"
                f" / {abs(angle_deg):.0f} deg",
                end="", flush=True
            )

            time.sleep(0.05)

        self.stop()
        print(
            f"\nRotation completed: "
            f"{math.degrees(total):.2f} deg"
        )
        time.sleep(0.5)

    def move(self, distance):
        start_x = self.x
        start_y = self.y
        heading = self.yaw

        speed = 0.0
        start = time.monotonic()

        print(f"\nMoving {distance:.2f} m")

        while rclpy.ok():
            self.update()

            if time.monotonic() - start > 600:
                raise RuntimeError("Movement timeout")

            dx = self.x - start_x
            dy = self.y - start_y

            travelled = (
                dx * math.cos(heading)
                + dy * math.sin(heading)
            )

            remaining = distance - travelled

            # No early stopping tolerance
            if remaining <= 0.0:
                break

            # Obstacle safety check
            if self.front_distance is None:
                raise RuntimeError("No valid front scan")

            if self.front_distance < 0.40:
                raise RuntimeError(
                    f"Obstacle ahead: {self.front_distance:.2f} m"
                )

            # Gradual deceleration
            desired = min(
                self.linear_speed,
                max(0.015, math.sqrt(0.10 * remaining))
            )

            if speed < desired:
                speed = min(speed + 0.004, desired)
            else:
                speed = max(speed - 0.004, desired)

            # Heading correction
            error = normalize(heading - self.yaw)

            correction = max(
                -0.15,
                min(0.15, 1.5 * error)
            )

            self.command(
                linear=speed,
                angular=correction
            )

            print(
                f"\rMoved: {travelled:.3f}"
                f" / {distance:.3f} m",
                end="", flush=True
            )

            time.sleep(0.05)

        self.stop()
        print(f"\nMovement completed: {travelled:.3f} m")
        time.sleep(0.5)

    def run_route(self):
        self.wait_for_data()

        print("\nStarting no-tolerance SLAM route")

        # Initial 360-degree scan
        self.rotate(360)

        # Four sides, three 90-degree turns
        for i in range(4):
            print(f"\n--- Side {i + 1}/4 ---")
            self.move(self.distance)

            if i < 3:
                self.rotate(90)

        self.stop()

        print("\nROUTE COMPLETED")
        print(
            f"Final odometry: "
            f"x={self.x:.3f}, y={self.y:.3f}"
        )


def main():
    parser = argparse.ArgumentParser()

    parser.add_argument(
        "--distance", type=float, default=2.6
    )
    parser.add_argument(
        "--linear-speed", type=float, default=0.10
    )
    parser.add_argument(
        "--angular-speed", type=float, default=0.15
    )

    args = parser.parse_args()

    if not 0 < args.distance <= 10:
        parser.error("Invalid distance")
    if not 0 < args.linear_speed <= 0.3:
        parser.error("Invalid linear speed")
    if not 0 < args.angular_speed <= 0.5:
        parser.error("Invalid angular speed")

    rclpy.init()

    node = MoveRotate(
        args.distance,
        args.linear_speed,
        args.angular_speed
    )

    try:
        node.run_route()

    except KeyboardInterrupt:
        print("\nInterrupted")

    except Exception as error:
        print(f"\nEmergency stop: {error}")

    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
