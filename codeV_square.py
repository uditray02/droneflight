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
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)
        self.velocity_pub = self.create_publisher(TwistStamped, "mavros/setpoint_velocity/cmd_vel", 10)
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")

        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz = 1/0.05

        self.velocity = TwistStamped()
#square
        self.waypoints = [
            (10.0, 0.0),  #fwd 10 meters
            (0.0, 10.0),  # right 10 meters
            (-10.0, 0.0),  # Turn right -10 meters (back)
            (0.0, -10.0)   # Turn right, -10 meters (back to start)
        ]
        
        self.current_waypoint = 0
        self.last_req = self.get_clock().now()

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        if not self.current_state.connected:
            return

        if self.current_state.mode != "OFFBOARD" and (self.get_clock().now() - self.last_req).nanoseconds / 1e9 > 5.0:
            set_mode_req = SetMode.Request()
            set_mode_req.custom_mode = 'OFFBOARD'
            if self.set_mode_client.call_async(set_mode_req).result():
                self.get_logger().info("OFFBOARD enabled")
            self.last_req = self.get_clock().now()

        elif not self.current_state.armed and (self.get_clock().now() - self.last_req).nanoseconds / 1e9 > 5.0:
            arm_cmd = CommandBool.Request()
            arm_cmd.value = True
            if self.arming_client.call_async(arm_cmd).result():
                self.get_logger().info("Vehicle armed")
            self.last_req = self.get_clock().now()

        if self.current_waypoint < len(self.waypoints):
            target_x, target_y = self.waypoints[self.current_waypoint]
            self.move_to_waypoint(target_x, target_y)

    def move_to_waypoint(self, target_x, target_y):
        error_x = target_x - self.velocity.twist.linear.x
        error_y = target_y - self.velocity.twist.linear.y

        self.velocity.twist.linear.x += 0.1 * error_x  
        self.velocity.twist.linear.y += 0.1 * error_y 


        
        if abs(error_x) < 0.1 and abs(error_y) < 0.1:
            self.current_waypoint += 1
            self.get_logger().info(f"Waypoint {self.current_waypoint} reached.")

        self.velocity_pub.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)
    offboard_node = OffboardNode()
    rclpy.spin(offboard_node)
    offboard_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
