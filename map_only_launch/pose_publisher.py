#!/usr/bin/env python3
"""
Standalone ROS2 publisher for TurtleBot4 mock data.
Publishes sample goals, dock status, velocity commands, odometry data,
and mock humans for proxemic visualization.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import time
import json
from typing import List, Dict

# ROS2 message imports
from geometry_msgs.msg import PoseStamped, Twist, Pose, PoseArray
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from irobot_create_msgs.msg import DockStatus


class pose_publisher(Node):
    """
    Publishes mock data to various TurtleBot4 topics.
    """

    def __init__(self):
        super().__init__('turtlebot4_mock_publisher')

        # rosbridge-compatible QoS (NO TRANSIENT_LOCAL)
        self.qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Humans should be BEST_EFFORT for rosbridge
        self.qos_humans = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self._init_publishers()
        self.get_logger().info('Mock publisher initialized')

    # ---------------------------------------------------------
    # Publishers
    # ---------------------------------------------------------
    def _init_publishers(self):

        self.goal_pose_pub = self.create_publisher(
            PoseStamped, '/gary/goal_pose', self.qos_reliable
        )
        self.get_logger().info('Created publisher: /gary/goal_pose')

        self.rule_output_pub = self.create_publisher(
            String, '/rule_output', self.qos_reliable
        )
        self.get_logger().info('Created publisher: /rule_output')

        self.dock_status_pub = self.create_publisher(
            DockStatus, '/dock_status', self.qos_reliable
        )
        self.get_logger().info('Created publisher: /dock_status')

        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', self.qos_reliable
        )
        self.get_logger().info('Created publisher: /cmd_vel')

        self.odom_pub = self.create_publisher(
            Odometry, '/odom', self.qos_reliable
        )
        self.get_logger().info('Created publisher: /odom')

        # Humans publisher (BEST_EFFORT)
        self.humans_pub = self.create_publisher(
            PoseArray, '/humans', self.qos_humans
        )
        self.get_logger().info('Created publisher: /humans')

    # ---------------------------------------------------------
    # HUMANS
    # ---------------------------------------------------------
    def publish_mock_humans(self):
        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # Updated human positions
        humans = [
            (4.0, 6.5),
            (9.0, 15.0)
        ]

        for x, y in humans:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            pose.orientation.w = 1.0
            msg.poses.append(pose)

        self.humans_pub.publish(msg)
        self.get_logger().info(f"Published {len(humans)} mock humans")

    # ---------------------------------------------------------
    # ODOMETRY
    # ---------------------------------------------------------
    def publish_odometry(self, x=0.0, y=0.0, z=0.0):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'

        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.w = 1.0

        self.odom_pub.publish(msg)
        self.get_logger().info(f"Published odometry: x={x}, y={y}")

    def publish_odometry_sequence(self, positions, interval_sec=1.0):
        self.get_logger().info(f"Publishing {len(positions)} odometry positions...")
        for pos in positions:
            self.publish_odometry(pos['x'], pos['y'], pos['z'])
            time.sleep(interval_sec)
        self.get_logger().info("Odometry sequence complete")

    # ---------------------------------------------------------
    # GOALS, RULES, DOCK, STOP 
    # ---------------------------------------------------------
    def publish_sample_goals(self, interval_sec=3.0):
        sample_goals = [
            {"position": {"x": 1.0, "y": 1.0}, "goal_type": "intermediate",
             "rule": "ZONE: NEAR, DIRECTION: FRONT_LEFT, OUTPUT: SLIGHT_LEFT"},
            {"position": {"x": 2.0, "y": 2.0}, "goal_type": "intermediate",
             "rule": "ZONE: MEDIUM, DIRECTION: FRONT, OUTPUT: STRAIGHT"},
            {"position": {"x": 3.0, "y": 3.0}, "goal_type": "global",
             "rule": "ZONE: FAR, DIRECTION: FRONT_RIGHT, OUTPUT: FINAL_APPROACH"}
        ]

        self.get_logger().info(f"Publishing {len(sample_goals)} sample goals...")

        for idx, goal in enumerate(sample_goals):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = 'map'
            pose_msg.pose.position.x = goal["position"]["x"]
            pose_msg.pose.position.y = goal["position"]["y"]
            pose_msg.pose.orientation.w = 1.0

            self.goal_pose_pub.publish(pose_msg)
            self.get_logger().info(
                f"Published goal {idx+1}/{len(sample_goals)}: "
                f"x={goal['position']['x']}, y={goal['position']['y']}"
            )

            rule_msg = String()
            rule_msg.data = json.dumps({
                "goal_type": goal["goal_type"],
                "rule": goal["rule"],
                "position": goal["position"],
                "timestamp": time.time()
            })
            self.rule_output_pub.publish(rule_msg)

            if idx < len(sample_goals) - 1:
                time.sleep(interval_sec)

        self.get_logger().info("All sample goals published successfully")

    def publish_dock_status(self, is_docked=True):
        msg = DockStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.is_docked = is_docked
        msg.dock_visible = is_docked
        self.dock_status_pub.publish(msg)
        self.get_logger().info(f"Published dock status: {is_docked}")

    def publish_stop_command(self):
        msg = Twist()
        self.cmd_vel_pub.publish(msg)
        self.get_logger().info("Published STOP command to /cmd_vel")


def main(args=None):
    """Main function to run the publisher."""
    rclpy.init(args=args)
    
    publisher = pose_publisher()
    
    try:
        # Example usage: publish sample goals
        publisher.get_logger().info('Starting sample goal publishing sequence...')
        publisher.publish_sample_goals(interval_sec=3.0)

        # Example: publish mock humans for testing human detection callbacks
        publisher.get_logger().info('\nPublishing mock humans...')
        publisher.publish_mock_humans() 
        
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
