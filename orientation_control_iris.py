import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Quaternion  # Added Quaternion import
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode
import math

current_state = State()

class OrientationControlNode(Node):
    def __init__(self):
        super().__init__("orientation_control_node")

        # Subscriber to MAVROS state
        self.state_sub = self.create_subscription(State, "mavros/state", self.state_cb, 10)

        # Publisher for pose with orientation setpoint
        self.local_pos_pub = self.create_publisher(PoseStamped, "mavros/setpoint_position/local", 10)

        # Service clients for arming and mode change
        self.arming_client = self.create_client(CommandBool, "/mavros/cmd/arming")
        self.set_mode_client = self.create_client(SetMode, "/mavros/set_mode")

        # Wait for services to be available
        self.wait_for_services()

        # Timer to control setpoint publishing rate
        self.rate = self.create_timer(0.05, self.run_loop)

        # Pose initialization (Position and Orientation control)
        self.pose = PoseStamped()
        self.pose.pose.position.x = 10.0
        self.pose.pose.position.y = 3.0
        self.pose.pose.position.z = 2.0

        # Orientation initialization (Quaternions for controlling rotation)
        self.pose.pose.orientation = Quaternion()
        self.pose.pose.orientation.x = 0.0
        self.pose.pose.orientation.y = 0.0
        self.pose.pose.orientation.z = math.sin(math.radians(90))  # 45 degrees of yaw
        self.pose.pose.orientation.w = math.cos(math.radians(90))

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

        # Publish orientation setpoint
        self.local_pos_pub.publish(self.pose)

def main(args=None):
    rclpy.init(args=args)

    # Create and spin the node
    orientation_control_node = OrientationControlNode()
    try:
        rclpy.spin(orientation_control_node)
    except KeyboardInterrupt:
        orientation_control_node.get_logger().info('Shutting down node...')
    finally:
        orientation_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
