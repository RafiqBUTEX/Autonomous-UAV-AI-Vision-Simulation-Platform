import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State


class OffboardControl(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.current_state = State()
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_cb, qos)

        self.current_pose = PoseStamped()
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_cb, qos)

        self.local_pos_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', qos)

        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # --- Waypoint sequence: simple square pattern at 2.5m altitude ---
        self.waypoints = [
            (0.0, 0.0, 2.5),   # takeoff / hover
            (3.0, 0.0, 2.5),   # forward
            (3.0, 3.0, 2.5),   # right
            (0.0, 3.0, 2.5),   # back
            (0.0, 0.0, 2.5),   # left (return to start)
        ]
        self.current_wp_index = 0
        self.arrival_threshold = 0.3  # meters
        self.hold_counter = 0
        self.hold_cycles_required = 40  # ~2 sec hold at each waypoint before advancing

        self.target_pose = PoseStamped()
        self._set_target_from_waypoint(self.waypoints[0])

        self.setpoint_count = 0
        self.last_mode_request_time = None
        self.last_arm_request_time = None
        self.request_cooldown_sec = 3.0

        self.mission_complete = False

        self.timer = self.create_timer(0.05, self.timer_cb)

    def state_cb(self, msg):
        self.current_state = msg

    def pose_cb(self, msg):
        self.current_pose = msg

    def _set_target_from_waypoint(self, wp):
        x, y, z = wp
        self.target_pose.pose.position.x = x
        self.target_pose.pose.position.y = y
        self.target_pose.pose.position.z = z

    def _distance_to_target(self):
        dx = self.current_pose.pose.position.x - self.target_pose.pose.position.x
        dy = self.current_pose.pose.position.y - self.target_pose.pose.position.y
        dz = self.current_pose.pose.position.z - self.target_pose.pose.position.z
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def timer_cb(self):
        self.target_pose.header.stamp = self.get_clock().now().to_msg()
        self.local_pos_pub.publish(self.target_pose)
        self.setpoint_count += 1

        if self.setpoint_count < 40:
            return

        now = self.get_clock().now()

        if self.current_state.mode != "OFFBOARD":
            if self._cooldown_expired(self.last_mode_request_time, now):
                self.last_mode_request_time = now
                self.set_mode(mode="OFFBOARD")
            return

        if not self.current_state.armed:
            if self._cooldown_expired(self.last_arm_request_time, now):
                self.last_arm_request_time = now
                self.arm(True)
            return

        # Armed and in OFFBOARD -> run the waypoint sequence
        if self.mission_complete:
            return

        dist = self._distance_to_target()
        if dist < self.arrival_threshold:
            self.hold_counter += 1
            if self.hold_counter >= self.hold_cycles_required:
                self.hold_counter = 0
                if self.current_wp_index < len(self.waypoints) - 1:
                    self.current_wp_index += 1
                    wp = self.waypoints[self.current_wp_index]
                    self._set_target_from_waypoint(wp)
                    self.get_logger().info(
                        f'Advancing to waypoint {self.current_wp_index}: {wp}')
                else:
                    self.mission_complete = True
                    self.get_logger().info('Waypoint mission complete. Holding final position.')
        else:
            self.hold_counter = 0

    def _cooldown_expired(self, last_time, now):
        if last_time is None:
            return True
        elapsed = (now - last_time).nanoseconds / 1e9
        return elapsed >= self.request_cooldown_sec

    def set_mode(self, mode):
        if not self.set_mode_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('set_mode service not available')
            return
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.set_mode_client.call_async(req)
        future.add_done_callback(self.set_mode_response)

    def set_mode_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Set mode request acknowledged: {result.mode_sent}')
        except Exception as e:
            self.get_logger().warn(f'Set mode request had no clean response: {e}')

    def arm(self, value):
        if not self.arming_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('arming service not available')
            return
        req = CommandBool.Request()
        req.value = value
        future = self.arming_client.call_async(req)
        future.add_done_callback(self.arm_response)

    def arm_response(self, future):
        try:
            result = future.result()
            self.get_logger().info(f'Arm request acknowledged: {result.success}')
        except Exception as e:
            self.get_logger().warn(f'Arm request had no clean response: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
