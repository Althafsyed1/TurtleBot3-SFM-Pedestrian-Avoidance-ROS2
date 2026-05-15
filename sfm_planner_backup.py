import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math

class SFMPlanner(Node):
    def __init__(self):
        super().__init__('sfm_planner')

        # SFM Parameters (Helbing & Molnar 1995)
        self.k_goal = 1.5
        self.k_rep = 8.0
        self.sigma = 0.8
        self.max_vel = 0.22
        self.max_omega = 1.0
        self.obstacle_range = 2.5
        self.goal_threshold = 0.3
        self.danger_zone = 0.5  # meters, triggers directional avoidance

        # Robot state
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0
        self.goal_x = None
        self.goal_y = None
        self.scan_data = None
        self.goal_reached = False

        # QoS profiles
        goal_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.VOLATILE,
            reliability=ReliabilityPolicy.RELIABLE)

        amcl_qos = QoSProfile(
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE)

        # Subscribers
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose',
            self.amcl_callback, amcl_qos)
        self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_callback, goal_qos)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.1, self.compute_sfm)
        self.get_logger().info('SFM Planner node started')

    def amcl_callback(self, msg):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny, cosy)

    def scan_callback(self, msg):
        self.scan_data = msg

    def goal_callback(self, msg):
        self.goal_x = msg.pose.position.x
        self.goal_y = msg.pose.position.y
        self.goal_reached = False
        self.get_logger().info(
            f'Goal received: ({self.goal_x:.2f}, {self.goal_y:.2f})')

    def compute_sfm(self):
        if self.goal_x is None or self.scan_data is None:
            return

        dx_goal = self.goal_x - self.robot_x
        dy_goal = self.goal_y - self.robot_y
        dist_goal = math.sqrt(dx_goal**2 + dy_goal**2)

        if dist_goal < self.goal_threshold:
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
                self.goal_reached = True
            self.cmd_pub.publish(Twist())
            return

        # Attractive force toward goal
        fx = self.k_goal * dx_goal / max(dist_goal, 0.001)
        fy = self.k_goal * dy_goal / max(dist_goal, 0.001)

        # Scan based directional avoidance
        angle = self.scan_data.angle_min
        front_blocked = False
        left_blocked = False
        right_blocked = False
        back_blocked = False
        min_front = float('inf')

        for r in self.scan_data.ranges:
            if self.scan_data.range_min < r < self.obstacle_range:
                # Obstacle angle relative to robot
                rel_angle = angle

                # Compute repulsive force
                obs_angle = self.robot_yaw + angle
                obs_x = self.robot_x + r * math.cos(obs_angle)
                obs_y = self.robot_y + r * math.sin(obs_angle)
                dx_obs = self.robot_x - obs_x
                dy_obs = self.robot_y - obs_y
                dist_obs = max(math.sqrt(dx_obs**2 + dy_obs**2), 0.001)
                force = self.k_rep * math.exp(-dist_obs / self.sigma)
                fx += force * dx_obs / dist_obs
                fy += force * dy_obs / dist_obs

                # Classify obstacle direction relative to robot
                if r < self.danger_zone:
                    if -0.5 < rel_angle < 0.5:
                        front_blocked = True
                        min_front = min(min_front, r)
                    elif rel_angle >= 0.5:
                        left_blocked = True
                    elif rel_angle <= -0.5:
                        right_blocked = True
                    else:
                        back_blocked = True

            angle += self.scan_data.angle_increment

        # Convert total force to velocity commands
        desired_angle = math.atan2(fy, fx)
        angle_error = math.atan2(
            math.sin(desired_angle - self.robot_yaw),
            math.cos(desired_angle - self.robot_yaw))

        cmd = Twist()

        # Directional avoidance logic
        if front_blocked and not left_blocked:
            # Human coming from front, turn left
            cmd.linear.x = 0.05
            cmd.angular.z = self.max_omega
            self.get_logger().info('Human ahead, turning left')
        elif front_blocked and not right_blocked:
            # Human coming from front, turn right
            cmd.linear.x = 0.05
            cmd.angular.z = -self.max_omega
            self.get_logger().info('Human ahead, turning right')
        elif front_blocked and left_blocked and right_blocked:
            # Human coming from front and sides, move backward
            cmd.linear.x = -0.1
            cmd.angular.z = 0.0
            self.get_logger().info('Human all around, moving backward')
        elif left_blocked and not front_blocked:
            # Human from left, turn right
            cmd.linear.x = self.max_vel * 0.5
            cmd.angular.z = -self.max_omega * 0.5
            self.get_logger().info('Human from left, turning right')
        elif right_blocked and not front_blocked:
            # Human from right, turn left
            cmd.linear.x = self.max_vel * 0.5
            cmd.angular.z = self.max_omega * 0.5
            self.get_logger().info('Human from right, turning left')
        else:
            # No danger, follow SFM forces normally
            cmd.angular.z = max(-self.max_omega,
                               min(self.max_omega, 2.0 * angle_error))
            if abs(angle_error) < 0.5:
                cmd.linear.x = min(self.max_vel, 0.5 * dist_goal)
            else:
                cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

def main(args=None):
    rclpy.init(args=args)
    node = SFMPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
