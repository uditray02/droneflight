import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import CommandBool, SetMode


class VelocityControlNode(Node):
    def __init__(self):
        super().__init__('velocity_control_node')

        # Current state and altitude
        self.current_state = State()
        self.current_altitude = 0.0

        # Subscribers
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.altitude_sub = self.create_subscription(Altitude, 'mavros/global_position/rel_alt', self.altitude_cb, 10)

        # Publishers
        self.velocity_pub = self.create_publisher(TwistStamped, 'mavros/setpoint_velocity/cmd_vel', 10)

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

        # Waypoint control
        self.waypoints = [
            (1.0, 0.0),  # Move along +X
            (0.0, 1.0),  # Move along +Y
            (-1.0, 0.0),  # Move along -X
            (0.0, -1.0),  # Move along -Y
        ]
        self.current_waypoint = 0
        self.waypoint_duration = 5.0  # Duration at each waypoint (seconds)
        self.start_time = None

        # Velocity initialization
        self.velocity = TwistStamped()
        self.velocity.twist.linear.z = 0.0
        self.last_req = self.get_clock().now()

    def state_cb(self, msg):
        self.current_state = msg

    def altitude_cb(self, msg):
        self.current_altitude = msg.relative_alt

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

        if self.current_state.mode == "OFFBOARD":
            self.control_altitude()

            # Control movement along waypoints
            if self.start_time is None:
                self.start_time = now
            elapsed_time = (now - self.start_time).nanoseconds / 1e9
            if elapsed_time > self.waypoint_duration:
                self.current_waypoint = (self.current_waypoint + 1) % len(self.waypoints)
                self.start_time = now

            # Set velocity for the current waypoint
            vx, vy = self.waypoints[self.current_waypoint]
            self.velocity.twist.linear.x = vx
            self.velocity.twist.linear.y = vy

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

    def control_altitude(self):
        target_altitude = 10.0
        if self.current_altitude < target_altitude:
            self.velocity.twist.linear.z = 1.0
        elif self.current_altitude > target_altitude:
            self.velocity.twist.linear.z = -1.0
        else:
            self.velocity.twist.linear.z = 0.0

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
