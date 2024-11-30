import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
from std_srvs.srv import SetBool
from geometry_msgs.msg import Quaternion
import math

current_state = State()

class OffboardControlNode(Node):
    def __init__(self):
        super().__init__("offb_node_py")

        # Subscriber
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)

        # Publisher for position, velocity, and orientation
        self.local_pos_pub = self.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)
        self.velocity_pub = self.create_publisher(Twist, "mavros/setpoint_velocity/cmd_vel", 10)

        # Service clients
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services to be available
        self.wait_for_services()

        # Setpoint publishing rate (20 Hz)
        self.rate = self.create_timer(0.05, self.run_loop)

        # Pose initialization
        self.pose = PoseStamped()
        self.pose.pose.position.x = 10.0
        self.pose.pose.position.y = 3.0
        self.pose.pose.position.z = 2.0

        # Orientation initialization (quaternion)
        self.pose.pose.orientation = Quaternion()
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = math.sin(math.radians(45))  # 45 degrees of yaw
        self.pose.pose.orientation.w = math.cos(math.radians(45))

        # Velocity initialization
        self.velocity = Twist()
        self.velocity.linear.x = 1.0  # Set desired linear velocity in x direction
        self.velocity.linear.y = 0.0  # Set desired linear velocity in y direction
        self.velocity.linear.z = 0.0  # No vertical velocity
        self.velocity.angular.x = 0.0  # No angular velocity around x axis
        self.velocity.angular.y = 0.0  # No angular velocity around y axis
        self.velocity.angular.z = 0.1  # Set angular velocity in z direction (turning)

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
            return

        # Send a few setpoints before starting OFFBOARD mode
        if self.get_clock().now() - self.last_req < rclpy.duration.Duration(seconds=5.0):
            self.local_pos_pub.publish(self.pose)
            return

        # Set OFFBOARD mode
        if current_state.mode != "OFFBOARD":
            offb_set_mode = SetMode.Request()
            offb_set_mode.custom_mode = "OFFBOARD"

            if self.set_mode_client.call(offb_set_mode).mode_sent:
                self.get_logger().info("OFFBOARD enabled")
            self.last_req = self.get_clock().now()

        # Arm the vehicle if it is not armed
        if not current_state.armed:
            arm_cmd = CommandBool.Request()
            arm_cmd.value = True

            if self.arming_client.call(arm_cmd).success:
                self.get_logger().info("Vehicle armed")
            self.last_req = self.get_clock().now()

        # Publish velocity command
        self.velocity_pub.publish(self.velocity)

        # Publish position with orientation (pose)
        self.local_pos_pub.publish(self.pose)


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    offboard_control_node = OffboardControlNode()
    try:
        rclpy.spin(offboard_control_node)
    except KeyboardInterrupt:
        offboard_control_node.get_logger().info('Shutting down node...')
    finally:
        offboard_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
