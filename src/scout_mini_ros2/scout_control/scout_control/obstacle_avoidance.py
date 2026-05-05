#!/usr/bin/env python3
"""
Obstacle Avoidance Node for Scout Mini Robot

This ROS2 node implements reactive obstacle avoidance for the Scout Mini robot in a maze environment.
The robot navigates autonomously from start position (-8, 8) to goal position (8, -7), avoiding collisions
by detecting obstacles with LaserScan data and performing 90-degree turns to the side with more space.

Key Features:
- Real-time LaserScan processing with sector-based distance calculation
- Reactive navigation: move forward until obstacle detected at 0.9m frontal distance
- Turn selection: choose left or right based on lateral space (75°-90° and -90°--75° sectors)
- Fixed 10-second turns for consistent behavior
- Post-turn forward movement (2 seconds) to stabilize after turns
- Final approach: when within 2m of goal, turn right 90° and proceed to goal
- Logging: periodic status logs every second for debugging

States:
- FORWARD: Move forward at 0.4 m/s until frontal obstacle < 0.9m
- STOP: Stop and decide turn direction based on left/right distances
- TURN: Turn in place for 10 seconds (100 cycles at 10Hz)
- POST_TURN_FORWARD: Move forward for 2 seconds after turn
- FINAL_TURN: Stop when near goal and prepare final turn
- FINAL_MOVE: Turn right for 10 seconds
- GOAL: Move towards goal with simple heading correction

Parameters:
- Front sector: ±10° for narrow frontal detection
- Left sector: 75°-90° for extreme left detection
- Right sector: -90°--75° for extreme right detection
- Safe distance: 0.9m frontal
- Speeds: 0.4 m/s linear, 1.5 rad/s angular
- Turn timeout: 10 seconds (100 cycles)

Dependencies:
- ROS2 Jazzy
- sensor_msgs/LaserScan
- nav_msgs/Odometry
- geometry_msgs/Twist

Usage:
1. Launch simulation: ros2 launch scout_sim scout_sim_maze.launch.py
2. Run node: ros2 run scout_control obstacle_avoidance

Author: Generated with GitHub Copilot
Date: April 21, 2026
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math
import numpy as np

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')

        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Timer for logging
        self.log_timer = self.create_timer(1.0, self.log_status)

        # Robot state
        self.state = 'FORWARD'  # States: FORWARD, STOP, TURN, POST_TURN_FORWARD, FINAL_TURN, GOAL
        self.linear_speed = 0.3 # m/s  # Doubled speed
        self.angular_speed = 1.5  # rad/s  # Slightly reduced for better precision

        # Obstacle detection
        self.safe_distance = 1.0  # meters  # Adjusted distance
        self.front_distance = float('inf')
        self.left_distance = float('inf')
        self.right_distance = float('inf')

        # Turning
        self.turn_angle = math.pi / 2  # 90 degrees
        self.initial_yaw = 0.0
        self.target_yaw = 0.0
        self.turn_direction = 0  # 1 for left, -1 for right

        # Post-turn forward
        self.post_turn_cycles = 0
        self.post_turn_duration = 2  # 2 seconds at 10Hz

        # Turn timeout
        self.turn_cycles = 0
        self.turn_timeout = 160  # 5 seconds at 10Hz

        # Position
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_yaw = 0.0

        # Goal
        self.goal_x = 8.0
        self.goal_y = -7.0
        self.goal_threshold = 1.0  # meters

        # Final approach
        self.final_approach_distance = 2.0  # meters from goal

        self.get_logger().info('Obstacle Avoidance Node initialized')

    def scan_callback(self, msg):
        # Process LaserScan data
        ranges = np.array(msg.ranges)
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        # Define sectors (in radians) - Narrow lateral sectors for extreme side detection
        front_angles = np.arange(-math.pi/18, math.pi/18, angle_increment)  # Narrowed to ±10°
        left_angles = np.arange(5*math.pi/12, math.pi/2, angle_increment)  # 75° to 90°
        right_angles = np.arange(-math.pi/2, -5*math.pi/12, angle_increment)  # -90° to -75°

        # Get indices
        front_indices = ((front_angles - angle_min) / angle_increment).astype(int)
        left_indices = ((left_angles - angle_min) / angle_increment).astype(int)
        right_indices = ((right_angles - angle_min) / angle_increment).astype(int)

        # Filter valid ranges
        front_ranges = ranges[front_indices]
        left_ranges = ranges[left_indices]
        right_ranges = ranges[right_indices]

        front_ranges = front_ranges[np.isfinite(front_ranges)]
        left_ranges = left_ranges[np.isfinite(left_ranges)]
        right_ranges = right_ranges[np.isfinite(right_ranges)]

        # Calculate minimum distances
        self.front_distance = np.min(front_ranges) if len(front_ranges) > 0 else float('inf')
        self.left_distance = np.min(left_ranges) if len(left_ranges) > 0 else float('inf')
        self.right_distance = np.min(right_ranges) if len(right_ranges) > 0 else float('inf')

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y

        # Get yaw from quaternion
        orientation = msg.pose.pose.orientation
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        twist = Twist()

        if self.state == 'FORWARD':
            if self.front_distance <= self.safe_distance:
                self.state = 'STOP'
                self.get_logger().info('Obstacle detected, stopping')
            else:
                twist.linear.x = self.linear_speed
                # Check if near goal for final approach
                distance_to_goal = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
                if distance_to_goal <= self.final_approach_distance:
                    self.state = 'FINAL_TURN'
                    self.get_logger().info('Near goal, preparing final turn')

        elif self.state == 'STOP':
            # Stop and choose turn direction
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            # Choose direction with more space
            self.get_logger().info(f'Left distance: {self.left_distance:.2f}, Right distance: {self.right_distance:.2f}')
            if self.left_distance > self.right_distance:
                self.turn_direction = 1  # left
            else:
                self.turn_direction = -1  # right
            self.initial_yaw = self.current_yaw
            self.target_yaw = self.initial_yaw + self.turn_direction * self.turn_angle
            self.state = 'TURN'
            self.turn_cycles = 0  # Reset turn counter
            self.get_logger().info(f'Turning {"left" if self.turn_direction == 1 else "right"}')

        elif self.state == 'TURN':
            # Turn for 5 seconds
            if self.turn_cycles < self.turn_timeout:
                twist.angular.z = self.turn_direction * self.angular_speed
                self.turn_cycles += 1
            else:
                self.post_turn_cycles = 0
                self.state = 'POST_TURN_FORWARD'
                self.get_logger().info('Girou noventa graus (5 seconds)')

        elif self.state == 'POST_TURN_FORWARD':
            # Move forward for 2 seconds after turn
            if self.post_turn_cycles < self.post_turn_duration:
                twist.linear.x = self.linear_speed
                self.post_turn_cycles += 1
            else:
                self.state = 'FORWARD'
                self.get_logger().info('Post-turn forward completed, resuming cycle')

        elif self.state == 'FINAL_TURN':
            # Stop and turn right 90 degrees
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.turn_direction = -1  # right
            self.initial_yaw = self.current_yaw
            self.target_yaw = self.initial_yaw + self.turn_direction * self.turn_angle
            self.state = 'FINAL_MOVE'
            self.turn_cycles = 0  # Reset turn counter
            self.get_logger().info('Final turn right')

        elif self.state == 'FINAL_MOVE':
            # Turn for 5 seconds
            if self.turn_cycles < self.turn_timeout:
                twist.angular.z = self.turn_direction * self.angular_speed
                self.turn_cycles += 1
            else:
                self.state = 'GOAL'
                self.get_logger().info('Final turn completed (5 seconds), moving to goal')

        elif self.state == 'GOAL':
            # Move towards goal
            distance_to_goal = math.sqrt((self.current_x - self.goal_x)**2 + (self.current_y - self.goal_y)**2)
            if distance_to_goal > self.goal_threshold:
                twist.linear.x = self.linear_speed
                # Simple heading correction towards goal
                goal_angle = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)
                yaw_error = self.normalize_angle(goal_angle - self.current_yaw)
                twist.angular.z = 0.5 * yaw_error
            else:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.get_logger().info('Goal reached!')

        self.cmd_vel_pub.publish(twist)

    def log_status(self):
        self.get_logger().info(
            f'State: {self.state}, Pos: ({self.current_x:.2f}, {self.current_y:.2f}), Yaw: {self.current_yaw:.2f}, '
            f'Front: {self.front_distance:.2f}, Left: {self.left_distance:.2f}, Right: {self.right_distance:.2f}'
        )

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()