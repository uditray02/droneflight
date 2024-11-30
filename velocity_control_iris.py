import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

current_state = State()

class VelocityControlNode(Node):
    def __init__(self):
        super().__init__("velocity_control_node")

        # Subscriber to MAVROS state
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)

        # Publisher for velocity setpoint
        self.velocity_pub = self.create_publisher(TwistStamped, "mavros/setpoint_velocity/cmd_vel", 10)

        # Service clients for arming and mode change
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services to be available
        self.wait_for_services()

        # Timer to control setpoint publishing rate
        self.rate = self.create_timer(0.05, self.run_loop)

        # Velocity initialization (Linear and Angular)
        self.velocity = TwistStamped()
        self.velocity.twist.linear.x = 20.0  # Desired linear velocity along X
        self.velocity.twist.linear.y = 0.0  # No velocity along Y
        self.velocity.twist.linear.z = 1.0  # No vertical velocity
        self.velocity.twist.angular.x = 0.0  # No angular velocity around X
        self.velocity.twist.angular.y = 0.0  # No angular velocity around Y
        self.velocity.twist.angular.z = 0.0  # Desired angular velocity around Z (yaw)

        self.last_req = self.get_clock().now()

    def wait_for_services(self):
        # Wait for arming and set_mode services
        while not self.arming_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("Arming service not available, waiting...")

        while not self.set_mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().info("SetMode service not available, waiting...")

    def state_cb(self, msg):
        global current_state
        current_state = msg

    def run_loop(self):
        # Wait until connected to the flight controller
        if not current_state.connected:
            self.get_logger().warn("Not connected to vehicle!")
            return

        # Send a few setpoints before starting OFFBOARD mode
        if self.get_clock().now() - self.last_req < rclpy.duration.Duration(seconds=5.0):
            self.velocity_pub.publish(self.velocity)
            self.get_logger().info("Publishing velocity setpoint...")
            return

        # Set OFFBOARD mode if not already in OFFBOARD
        if current_state.mode != "OFFBOARD":
            offb_set_mode = SetMode.Request()
            offb_set_mode.custom_mode = "OFFBOARD"

            if self.set_mode_client.call(offb_set_mode).mode_sent:
                self.get_logger().info("OFFBOARD mode enabled")
            else:
                self.get_logger().warn("Failed to set OFFBOARD mode")

            self.last_req = self.get_clock().now()

        # Arm the vehicle if not already armed
        if not current_state.armed:
            arm_cmd = CommandBool.Request()
            arm_cmd.value = True

            if self.arming_client.call(arm_cmd).success:
                self.get_logger().info("Vehicle armed")
            else:
                self.get_logger().warn("Failed to arm the vehicle")

            self.last_req = self.get_clock().now()

        # Publish velocity setpoint
        self.velocity_pub.publish(self.velocity)

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    velocity_control_node = VelocityControlNode()
    try:
        rclpy.spin(velocity_control_node)
    except KeyboardInterrupt:
        velocity_control_node.get_logger().info('Shutting down node...')
    finally:
        velocity_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
