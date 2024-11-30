import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode


class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')

        self.current_state = State()

        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10
        )

        self.velocity_pub = self.create_publisher(
            TwistStamped,
            'mavros/setpoint_velocity/cmd_vel',
            10
        )

        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')
        self.get_logger().info("Waiting for services...")
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.get_logger().info("Services are available.")
        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Velocity initialization (Linear and Angular)
        self.velocity = TwistStamped()
        self.velocity.twist.linear.x = float(1.0)  # Desired linear velocity along X
        self.velocity.twist.linear.y = float(0.0)  # No velocity along Y
        self.velocity.twist.linear.z = float(0.0)  # Desired vertical velocity
        self.velocity.twist.angular.x = float(0.0)  # No angular velocity around X
        self.velocity.twist.angular.y = float(0.0)  # No angular velocity around Y
        self.velocity.twist.angular.z = float(0.0)  # Desired angular velocity around Z (yaw)

        self.last_req = self.get_clock().now()

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        now = self.get_clock().now()
        if not self.current_state.connected:
            self.get_logger().warn("Not connected to vehicle!")
            return

        if self.current_state.mode != "OFFBOARD" and (now - self.last_req).nanoseconds > 5e9:
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.offboard_mode_response)
            self.last_req = now

        elif not self.current_state.armed and (now - self.last_req).nanoseconds > 5e9:
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response)
            self.last_req = now

        self.velocity_pub.publish(self.velocity)

    def offboard_mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode enabled")
            else:
                self.get_logger().warn("Failed to enable OFFBOARD mode")
        except Exception as e:
            self.get_logger().error(f"Error enabling OFFBOARD mode: {str(e)}")

    def arm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed")
            else:
                self.get_logger().warn("Failed to arm the vehicle")
        except Exception as e:
            self.get_logger().error(f"Error arming vehicle: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    velocity_control_node = VelocityControlNode()
    try:
        rclpy.spin(velocity_control_node)
    except KeyboardInterrupt:
        velocity_control_node.get_logger().info('Shutting down node...')
    finally:
        velocity_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
