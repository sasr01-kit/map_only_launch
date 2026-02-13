#!/usr/bin/env python3
"""
Standalone ROS2 publisher for TurtleBot4 mock data.
Publishes sample goals, dock status, velocity commands, and odometry data.
Creates topics if they don't exist yet.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import time
import json
from typing import List, Dict

# ROS2 message imports
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import DockStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header


class pose_publisher(Node):
    """
    Publishes mock data to various TurtleBot4 topics.
    Creates topics automatically if they don't exist.
    """
    
    def __init__(self):
        super().__init__('turtlebot4_mock_publisher')
        
        # QoS profile for reliable communication
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # Initialize all publishers
        self._init_publishers()
        
        self.get_logger().info('TurtleBot4 Mock Publisher initialized')
        self.get_logger().info('All topics created and ready for publishing')
    
    def _init_publishers(self):
        """Initialize all publishers for different topics."""
        
        # Goal pose publisher (/gary/goal_pose)
        self.goal_pose_pub = self.create_publisher(
            PoseStamped,
            '/gary/goal_pose',
            self.qos_profile
        )
        self.get_logger().info('Created publisher: /gary/goal_pose')
        
        # Rule output publisher (/rule_output)
        self.rule_output_pub = self.create_publisher(
            String,
            '/rule_output',
            self.qos_profile
        )
        self.get_logger().info('Created publisher: /rule_output')
        
        # Dock status publisher (/dock_status)
        self.dock_status_pub = self.create_publisher(
            DockStatus,
            '/dock_status',
            self.qos_profile
        )
        self.get_logger().info('Created publisher: /dock_status')
        
        # Velocity command publisher (/cmd_vel)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            self.qos_profile
        )
        self.get_logger().info('Created publisher: /cmd_vel')
        
        # Odometry publisher (/odom) - for testing pose callbacks
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            self.qos_profile
        )
        self.get_logger().info('Created publisher: /odom')
    
    def publish_sample_goals(self, interval_sec: float = 3.0) -> None:
        """
        Publishes 3 sample goal poses that form a line on the map.
        - First two goals: intermediate waypoints
        - Third goal: global goal
        
        For each goal, also publishes a corresponding rule output string.
        
        Args:
            interval_sec: Time to wait between publishing each goal (default: 3 seconds)
        """
        # Define 3 points that form a line
        sample_goals = [
            {
                "position": {"x": 1.0, "y": 1.0},
                "goal_type": "intermediate",
                "rule": "ZONE: NEAR, DIRECTION: FRONT_LEFT, OUTPUT: SLIGHT_LEFT"
            },
            {
                "position": {"x": 2.0, "y": 2.0},
                "goal_type": "intermediate",
                "rule": "ZONE: MEDIUM, DIRECTION: FRONT, OUTPUT: STRAIGHT"
            },
            {
                "position": {"x": 3.0, "y": 3.0},
                "goal_type": "global",
                "rule": "ZONE: FAR, DIRECTION: FRONT_RIGHT, OUTPUT: FINAL_APPROACH"
            }
        ]
        
        self.get_logger().info(f'Publishing {len(sample_goals)} sample goals...')
        
        for idx, goal in enumerate(sample_goals):
            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = goal["position"]["x"]
            pose_msg.pose.position.y = goal["position"]["y"]
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0
            
            # Publish goal pose
            self.goal_pose_pub.publish(pose_msg)
            self.get_logger().info(
                f"Published goal {idx + 1}/{len(sample_goals)}: "
                f"x={goal['position']['x']:.2f}, y={goal['position']['y']:.2f}, "
                f"type='{goal['goal_type']}'"
            )
            
            # Create rule output message (JSON format)
            rule_output_data = {
                "goal_type": goal["goal_type"],
                "rule": goal["rule"],
                "position": goal["position"],
                "timestamp": time.time()
            }
            
            rule_msg = String()
            rule_msg.data = json.dumps(rule_output_data)
            
            # Publish rule output
            self.rule_output_pub.publish(rule_msg)
            self.get_logger().info(
                f"Published rule output: goal_type='{goal['goal_type']}', "
                f"rule='{goal['rule']}'"
            )
            
            # Wait before publishing next goal (except after last goal)
            if idx < len(sample_goals) - 1:
                self.get_logger().info(f'Waiting {interval_sec} seconds before next goal...')
                time.sleep(interval_sec)
        
        self.get_logger().info('All sample goals published successfully')
    
    def publish_dock_status(self, is_docked: bool = True) -> None:
        """
        Publishes a docking status to the /dock_status topic.
        
        Args:
            is_docked: Whether the robot is docked (True) or undocked (False)
        """
        dock_msg = DockStatus()
        dock_msg.header.stamp = self.get_clock().now().to_msg()
        dock_msg.header.frame_id = 'base_link'
        dock_msg.is_docked = is_docked
        dock_msg.dock_visible = is_docked
        
        self.dock_status_pub.publish(dock_msg)
        status = "docked" if is_docked else "undocked"
        self.get_logger().info(f'Published dock status: {status}')
    
    def publish_stop_command(self) -> None:
        """
        Publishes zero velocity to /cmd_vel to immediately stop the robot.
        """
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(stop_msg)
        self.get_logger().info('Published STOP command to /cmd_vel')
    
    def publish_velocity_command(self, linear_x: float = 0.0, angular_z: float = 0.0) -> None:
        """
        Publishes a velocity command to /cmd_vel.
        
        Args:
            linear_x: Linear velocity in x direction (m/s)
            angular_z: Angular velocity around z axis (rad/s)
        """
        vel_msg = Twist()
        vel_msg.linear.x = linear_x
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = angular_z
        
        self.cmd_vel_pub.publish(vel_msg)
        self.get_logger().info(
            f'Published velocity command: linear.x={linear_x:.2f}, angular.z={angular_z:.2f}'
        )
    
    def publish_odometry(self, x: float = 0.0, y: float = 0.0, z: float = 0.0) -> None:
        """
        Publishes mock odometry data to /odom.
        
        Args:
            x: X position (meters)
            y: Y position (meters)
            z: Z position (meters)
        """
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        # Set position
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.position.z = z
        
        # Set orientation (identity quaternion)
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = 0.0
        odom_msg.pose.pose.orientation.w = 1.0
        
        # Set velocities (zero for stationary)
        odom_msg.twist.twist.linear.x = 0.0
        odom_msg.twist.twist.angular.z = 0.0
        
        self.odom_pub.publish(odom_msg)
        self.get_logger().info(f'Published odometry: x={x:.2f}, y={y:.2f}, z={z:.2f}')
    
    def publish_odometry_sequence(self, positions: List[Dict[str, float]], interval_sec: float = 1.0) -> None:
        """
        Publishes a sequence of odometry positions.
        
        Args:
            positions: List of dictionaries with 'x', 'y', 'z' keys
            interval_sec: Time to wait between each position (default: 1 second)
        """
        self.get_logger().info(f'Publishing {len(positions)} odometry positions...')
        
        for idx, pos in enumerate(positions):
            self.publish_odometry(pos.get('x', 0.0), pos.get('y', 0.0), pos.get('z', 0.0))
            
            if idx < len(positions) - 1:
                time.sleep(interval_sec)
        
        self.get_logger().info('Odometry sequence complete')


def main(args=None):
    """Main function to run the publisher."""
    rclpy.init(args=args)
    
    publisher = pose_publisher()
    
    try:
        # Example usage: publish sample goals
        publisher.get_logger().info('Starting sample goal publishing sequence...')
        publisher.publish_sample_goals(interval_sec=3.0)
        
        # Example: publish some odometry positions along the path
        publisher.get_logger().info('\nPublishing odometry sequence...')
        odom_positions = [
            {'x': 0.5, 'y': 0.5, 'z': 0.0},
            {'x': 1.0, 'y': 1.0, 'z': 0.0},
            {'x': 1.5, 'y': 1.5, 'z': 0.0},
            {'x': 2.0, 'y': 2.0, 'z': 0.0},
            {'x': 2.5, 'y': 2.5, 'z': 0.0},
            {'x': 3.0, 'y': 3.0, 'z': 0.0},
        ]
        publisher.publish_odometry_sequence(odom_positions, interval_sec=1.0)
        
        # Example: publish dock status
        publisher.get_logger().info('\nPublishing undock status...')
        publisher.publish_dock_status(is_docked=False)
        
        time.sleep(2.0)
        
        publisher.get_logger().info('\nPublishing dock status...')
        publisher.publish_dock_status(is_docked=True)
        
        # Example: publish stop command
        time.sleep(1.0)
        publisher.get_logger().info('\nPublishing stop command...')
        publisher.publish_stop_command()
        
        publisher.get_logger().info('\n=== All publishing complete ===')
        publisher.get_logger().info('Publishers remain active. Press Ctrl+C to exit.')
        
        # Keep node alive to maintain publishers
        rclpy.spin(publisher)
        
    except KeyboardInterrupt:
        publisher.get_logger().info('Shutting down...')
    finally:
        publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
