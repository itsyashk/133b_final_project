#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DStarPlanner(Node):
    def __init__(self):
        super().__init__('d_star_planner')
        self.get_logger().info("D* Planner node started. Waiting for simulation to settle...")
        
        # Publisher to send velocity commands.
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Delay before starting the path (in seconds)
        self.pre_start_delay = 5.0  
        self.start_time = self.get_clock().now().nanoseconds / 1e9
        
        # Define the pre-planned path as a list of segments.
        # Each segment is a dictionary with:
        #   'linear': linear velocity (m/s)
        #   'angular': angular velocity (rad/s)
        #   'duration': time in seconds to hold that command
        self.path = [
            {'linear': 0.2, 'angular': 0.0,  'duration': 5.0},   # Move forward for 5 seconds
            {'linear': 0.0, 'angular': 0.5,  'duration': 3.0},   # Turn left for 3 seconds
            {'linear': 0.2, 'angular': 0.0,  'duration': 5.0},   # Move forward for 5 seconds
            {'linear': 0.0, 'angular': -0.5, 'duration': 3.0},   # Turn right for 3 seconds
            {'linear': 0.2, 'angular': 0.0,  'duration': 5.0},   # Move forward for 5 seconds
            {'linear': 0.0, 'angular': 0.0,  'duration': 2.0}    # Stop for 2 seconds
        ]
        self.current_segment_index = 0
        self.segment_start_time = None
        
        # Create a timer to update commands at 10 Hz.
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Wait for the initial delay before starting the path.
        if current_time - self.start_time < self.pre_start_delay:
            return
        
        # Initialize the segment start time if this is the first run.
        if self.segment_start_time is None:
            self.segment_start_time = current_time

        elapsed_time = current_time - self.segment_start_time
        
        if self.current_segment_index < len(self.path):
            segment = self.path[self.current_segment_index]
            
            # Check if the current segment's duration has elapsed.
            if elapsed_time >= segment['duration']:
                self.current_segment_index += 1
                if self.current_segment_index < len(self.path):
                    self.segment_start_time = current_time
                    segment = self.path[self.current_segment_index]
                    self.get_logger().info(f"Moving to segment {self.current_segment_index}: {segment}")
                else:
                    self.get_logger().info("Pre-planned path complete. Stopping robot.")
                    twist = Twist()
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0
                    self.cmd_pub.publish(twist)
                    self.destroy_timer(self.timer)
                    return

            # Publish the current segment's velocity command.
            twist = Twist()
            twist.linear.x = segment['linear']
            twist.angular.z = segment['angular']
            self.cmd_pub.publish(twist)
        else:
            # If no more segments, ensure the robot is stopped.
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            self.get_logger().info("No more segments. Robot stopped.")
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    node = DStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
