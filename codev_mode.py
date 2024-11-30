import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

class OffboardNode(Node):
    def __init__(self):
        super().__init__('offb_node_py')

        # Current state
        self.current_state = State()

        # Subscribers
        self.state_sub = self.create_subscription(
            State,
            'mavros/state',
            self.state_cb,
            10
        )

        # Publishers
        self.local_pos_pub = self.create_publisher(
            PoseStamped,
            'mavros/setpoint_position/local',
            10
        )

        # Service clients
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, 'mavros/set_mode')

        # Ensure services are available
        self.get_logger().info("Waiting for services...")
        self.arming_client.wait_for_service()
        self.set_mode_client.wait_for_service()
        self.get_logger().info("Services are available.")

        # Setpoint publishing rate
        self.timer_period = 0.05  # 20Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Control variables
        self.pose = PoseStamped()
        self.pose.pose.position.x = float(0)
        self.pose.pose.position.y = float(0)
        self.pose.pose.position.z = float(4)

        self.last_req = self.get_clock().now()
        self.offboard_enabled = False
        self.armed = False

    def state_cb(self, msg):
        self.current_state = msg

    def timer_callback(self):
        now = self.get_clock().now()

        # Enable OFFBOARD mode
        if self.current_state.mode != "OFFBOARD" and (now - self.last_req).nanoseconds > 5e9:
            req = SetMode.Request()
            req.custom_mode = "OFFBOARD"
            future = self.set_mode_client.call_async(req)
            future.add_done_callback(self.offboard_mode_response)
            self.last_req = now

        # Arm the vehicle
        elif not self.current_state.armed and (now - self.last_req).nanoseconds > 5e9:
            req = CommandBool.Request()
            req.value = True
            future = self.arming_client.call_async(req)
            future.add_done_callback(self.arm_response)
            self.last_req = now

        # Publish the setpoint
        self.local_pos_pub.publish(self.pose)

    def offboard_mode_response(self, future):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info("OFFBOARD mode enabled")
        except Exception as e:
            self.get_logger().error(f"Failed to set OFFBOARD mode: {str(e)}")

    def arm_response(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Vehicle armed")
        except Exception as e:
            self.get_logger().error(f"Failed to arm vehicle: {str(e)}")


def main(args=None):
    rclpy.init(args=args)

    offb_node = OffboardNode()

    try:
        rclpy.spin(offb_node)
    except KeyboardInterrupt:
        offb_node.get_logger().info("Shutting down node...")
    finally:
        offb_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
