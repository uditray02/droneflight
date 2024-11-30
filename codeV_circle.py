#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math


class OffboardNode(Node):

    def __init__(self):
        super().__init__("offboard_node_py")

        self.current_state = State()

        # Subscriber for MAVROS state
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)

        # Publisher for setpoint velocity
        self.velocity_pub = self.create_publisher(TwistStamped, "mavros/setpoint_velocity/cmd_vel", 10)

        # Service client for arming
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")

        # Service client for setting mode
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Setpoint publishing rate (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz = 1/0.05

        self.velocity = TwistStamped()

        # Parameters for the circular path
        self.radius = 10.0  # Radius of the circle in meters
        self.num_waypoints = 36  # Number of waypoints for the circle (higher for smoother circle)
        self.waypoints = self.generate_circular_waypoints(self.radius, self.num_waypoints)

        self.current_waypoint = 0
        self.last_req = self.get_clock().now()

    def generate_circular_waypoints(self, radius, num_points):
        """Generate waypoints for a circular path."""
        waypoints = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            waypoints.append((x, y))
        return waypoints

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        if not self.current_state.connected:
            return

        if self.current_state.mode != "OFFBOARD" and (self.get_clock().now() - self.last_req).nanoseconds / 1e9 > 5.0:
            # Set mode to OFFBOARD
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            if self.set_mode_client.call_async(set_mode_req).result():
                self.get_logger().info("OFFBOARD enabled")
            self.last_req = self.get_clock().now()

        elif not self.current_state.armed and (self.get_clock().now() - self.last_req).nanoseconds / 1e9 > 5.0:
            # Arm the vehicle
            arm_cmd = CommandBool.Request()
            arm_cmd.value = True
            if self.arming_client.call_async(arm_cmd).result():
                self.get_logger().info("Vehicle armed")
            self.last_req = self.get_clock().now()

        # Move to the current waypoint
        if self.current_waypoint < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint]
            self.move_to_waypoint(target_x, target_y)

    def move_to_waypoint(self, target_x, target_y):
        # Simple proportional control to move towards target point
        error_x = target_x - self.velocity.twist.linear.x
        error_y = target_y - self.velocity.twist.linear.y

        # Simple proportional control for velocity
        self.velocity.twist.linear.x += 0.1 * error_x  # Adjust forward velocity (x direction)
        self.velocity.twist.linear.y += 0.1 * error_y  # Adjust sideways velocity (y direction)

        # If close enough to target, move to next waypoint
        if abs(error_x) < 0.1 and abs(error_y) < 0.1:
            self.current_waypoint += 1
            self.get_logger().info(f"Waypoint {self.current_waypoint} reached.")

        self.velocity_pub.publish(self.velocity)


def main(args=None):
    rclpy.init(args=args)

    offboard_node = OffboardNode()

    # Setpoint publishing MUST be faster than 2Hz
    rclpy.spin(offboard_node)

    offboard_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
