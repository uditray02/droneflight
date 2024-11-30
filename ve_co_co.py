#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

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

        # Example velocity commands (units: m/s)
        self.velocity.twist.linear.x = 4.0  # Forward velocity
        self.velocity.twist.linear.y = 0.0  # Sideways velocity
        self.velocity.twist.linear.z = 0.0  # Upward/downward velocity
        self.velocity.twist.angular.x = 0.0 # Angular velocity about x-axis
        self.velocity.twist.angular.y = 0.0 # Angular velocity about y-axis
        self.velocity.twist.angular.z = 0.0 # Yaw rate

        # Wait for Flight Controller connection
        self.last_req = self.get_clock().now()

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
