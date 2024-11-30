import rclpy
from rclpy.node import Node
from mavros_msgs.msg import State, GlobalPositionTarget, PositionTarget
from geometry_msgs.msg import PoseStamped
import time

class TelemetryNode(Node):
    def __init__(self):
        super().__init__('telemetry_node')

        # Create a subscriber for the PX4 state
        self.state_subscriber = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10)
        
        # Create a subscriber for the global position (GPS) data
        self.gps_subscriber = self.create_subscription(
            GlobalPositionTarget, '/mavros/global_position/global', self.gps_callback, 10)

        # Create a publisher for the position target (to send commands to PX4)
        self.position_publisher = self.create_publisher(PositionTarget, '/mavros/setpoint_raw/target', 10)

        # Initialize the FCU connection URL
        self.fcu_url = 'udp:127.0.0.1:14557'

        # Store the latest GPS data
        self.latest_gps = None

        # Loop to periodically check for the telemetry
        self.timer = self.create_timer(1.0, self.telemetry_loop)

    def state_callback(self, msg):
        self.get_logger().info(f"Current State: {msg.mode}, Armed: {msg.armed}")

    def gps_callback(self, msg):
        # Store the latest GPS data
        self.latest_gps = msg
        self.get_logger().info(f"GPS Position: Lat: {msg.latitude}, Lon: {msg.longitude}, Alt: {msg.altitude}")

    def telemetry_loop(self):
        # If GPS data is available, print it in the telemetry loop
        if self.latest_gps:
            self.get_logger().info(f"Telemetry - GPS Position: Lat: {self.latest_gps.latitude}, Lon: {self.latest_gps.longitude}, Alt: {self.latest_gps.altitude}")

        # Simulate sending telemetry or commands to the autopilot (PX4)
        target_position = PositionTarget()
        target_position.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        target_position.type_mask = PositionTarget.IGNORE_VX + PositionTarget.IGNORE_VY + PositionTarget.IGNORE_VZ + PositionTarget.IGNORE_YAW
        target_position.position.x = 10.0  # Example target position in X (m)
        target_position.position.y = 0.0   # Example target position in Y (m)
        target_position.position.z = -10.0 # Example target position in Z (m)
        
        self.position_publisher.publish(target_position)
        self.get_logger().info("Publishing target position.")

def main(args=None):
    rclpy.init(args=args)
    telemetry_node = TelemetryNode()
    rclpy.spin(telemetry_node)
    telemetry_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()