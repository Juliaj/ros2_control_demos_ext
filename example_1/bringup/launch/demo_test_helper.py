# Copyright 2025 ros2_control Development Team
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# author: Julia Jia

import time
import rclpy
import signal
import sys
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from rclpy.parameter import Parameter
from nav_msgs.msg import Odometry #nav_msgs/msg/Odometry
import math
from std_srvs.srv import Empty
from sensor_msgs.msg import JointState

class OdometryTracker:
    '''
    This class is used to track the odometry of the robot by calculating the distance traveled 
    by the robot in a straight line and in an arc.
    '''
    def __init__(self, logger):
        self.logger = logger
        self.initial_position = None
        self.current_position = None
        self.last_orientation = None
        self.distance_traveled = 0.0
        self.arc_distance_traveled = 0.0
        self.last_timestamp = None
        self.min_dt_threshold = 0.02  # Minimum time delta to process (20ms)

    def update_odometry(self, msg):
        """Process new odometry message and update tracking metrics"""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        twist = msg.twist.twist
        current_time = self._get_current_time(msg)

        if self.initial_position is None:
            self._initialize_initial_position(position, orientation, current_time)
            return

        dt = self._calculate_time_delta(current_time)
        if dt < self.min_dt_threshold:
            return

        self._log_large_time_delta(dt)
        incremental_distance = self._calculate_incremental_distance(position)

        if incremental_distance > 0.001:
            self._update_distance_metrics(twist, dt, incremental_distance)
            self._log_distances(incremental_distance)

        self._update_position_data(position, orientation, current_time)

    def get_distance_traveled(self):
        return {
            'straight_line': self.distance_traveled,
            'arc': self.arc_distance_traveled
        }

    def _get_current_time(self, msg):
        """Get timestamp from odometry message"""
        return rclpy.time.Time.from_msg(msg.header.stamp)  # Use actual message timestamp

    def _initialize_initial_position(self, position, orientation, current_time):
        self.initial_position = position
        self.current_position = position
        self.last_orientation = orientation
        self.last_timestamp = current_time
        self.logger.info(f"Initial position set: x={position.x:.2f}, y={position.y:.2f}")

    def _calculate_time_delta(self, current_time):
        return (current_time - self.last_timestamp).nanoseconds / 1e9

    def _log_large_time_delta(self, dt):
        if dt > 0.5:
            self.logger.warn(f"Large time delta between odometry samples: {dt:.4f}s")

    def _calculate_incremental_distance(self, position):
        dx = position.x - self.current_position.x
        dy = position.y - self.current_position.y
        return math.sqrt(dx**2 + dy**2)

    def _update_distance_metrics(self, twist, dt, incremental_distance):
        self.distance_traveled += incremental_distance
        linear_speed = math.sqrt(twist.linear.x**2 + twist.linear.y**2)
        angular_speed = abs(twist.angular.z)

        if angular_speed > 0.05:
            turn_radius = linear_speed / angular_speed if angular_speed > 0 else 0
            angle_turned = angular_speed * dt
            arc_length = turn_radius * angle_turned
            self.arc_distance_traveled += max(arc_length, incremental_distance)
        else:
            self.arc_distance_traveled += incremental_distance

    def _log_distances(self, incremental_distance):
        if int(self.distance_traveled * 10) % 10 == 0 and incremental_distance > 0.01:
            self.logger.info(
                f"Distances - Straight: {self.distance_traveled:.2f}m, "
                f"Arc: {self.arc_distance_traveled:.2f}m"
            )

    def _update_position_data(self, position, orientation, current_time):
        self.current_position = position
        self.last_orientation = orientation
        self.last_timestamp = current_time

class SlipDetector:
    def __init__(self, logger, wheel_radius, wheel_joint_names):
        self.logger = logger
        self.wheel_radius = wheel_radius
        self.wheel_joint_names = wheel_joint_names
        self.wheel_traveled_distance = 0.0
        self.slip_ratio = 1.0
        self.last_wheel_positions = {}

    def update_wheel_rotation(self, msg):
        """Process joint state message to update wheel rotation metrics"""
        if not self.last_wheel_positions:
            self._initialize_wheel_positions(msg)
            return
            
        total_rotation = 0.0
        wheel_count = 0
        
        for i, name in enumerate(msg.name):
            if name in self.wheel_joint_names:
                current_pos = msg.position[i]
                if name in self.last_wheel_positions:
                    rotation = self._calculate_rotation(current_pos, name)
                    total_rotation += abs(rotation)
                    wheel_count += 1
                    self.last_wheel_positions[name] = current_pos
        
        if wheel_count > 0:
            self._update_wheel_distance(total_rotation, wheel_count)

    def calculate_slip(self, odometry_distance):
        """Calculate slip ratio based on wheel and odometry distances"""
        if odometry_distance < 0.1:
            return
            
        self.slip_ratio = self.wheel_traveled_distance / odometry_distance
        
        if self.slip_ratio > 1.1:
            self.logger.info(
                f"Wheel spin slip detected! Ratio: {self.slip_ratio:.3f} "
                f"(wheels moved {self.wheel_traveled_distance:.2f}m, "
                f"robot moved {odometry_distance:.2f}m)"
            )
            
    def _initialize_wheel_positions(self, msg):
        for i, name in enumerate(msg.name):
            if name in self.wheel_joint_names:
                self.last_wheel_positions[name] = msg.position[i]

    def _calculate_rotation(self, current_pos, name):
        rotation = current_pos - self.last_wheel_positions[name]
        if abs(rotation) > math.pi:
            if rotation > 0:
                rotation -= 2 * math.pi
            else:
                rotation += 2 * math.pi
        return rotation

    def _update_wheel_distance(self, total_rotation, wheel_count):
        avg_rotation = total_rotation / wheel_count
        incremental_distance = avg_rotation * self.wheel_radius
        self.wheel_traveled_distance += incremental_distance


class AckermannBotDemoTest(Node):
    '''
    This class is used to test the ackermann bot demo, get odometry and joint states and detect slip.
    '''
    def __init__(self, run_duration=600.0):
        super().__init__("ackermannbot_demo_helper_node")
        self.is_moving = False
        self.start_time = None
        self.run_duration = run_duration 

        self.publisher_ = self.create_publisher(TwistStamped, "/ackermann_steering_controller/reference", 10)
        
        # Create subscriber for odometry
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odometry',
            self.odom_callback,
            10)
            
        # Create subscriber for joint states to monitor wheel rotations
        self.joint_state_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)
        
        # Wheel slip detection variables
        wheel_joints = [
            'rear_left_wheel_joint',
            'rear_right_wheel_joint',
            'front_left_wheel_joint',
            'front_right_wheel_joint'
        ]
        self.slip_detector = SlipDetector(
            logger=self.get_logger(),
            wheel_radius=0.3,
            wheel_joint_names=wheel_joints
        )
        
        # Create the twist message as a class attribute - initially stopped
        self.twist_msg = self.create_twist_msg()
        self.cmd_timer = self.create_timer(0.1, lambda: self.publish_cmd(self.twist_msg))

        # Create OdometryTracker instance
        self.odometry_tracker = OdometryTracker(self.get_logger())
        
        # Timer for periodic slip analysis
        self.slip_timer = self.create_timer(1.0, self.analyze_slip)

        # Register shutdown service
        self.shutdown_service = self.create_service(Empty, 'shutdown_robot', self._shutdown_callback)


    def odom_callback(self, msg):
        """Handle new odometry messages by delegating to tracker"""
        self.odometry_tracker.update_odometry(msg)

    def publish_cmd(self, twist_msg):
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(twist_msg)
    
    def joint_state_callback(self, msg):
        """Handle joint states by delegating to slip detector"""
        self.slip_detector.update_wheel_rotation(msg)

    def start_moving(self, linear_speed=1.5, angular_speed=0.8):
        """Start the robot moving with the given speeds"""
        if self.is_moving:
            self.get_logger().info("Robot is already moving")
            return
            
        self.get_logger().info(f"Starting movement with linear={linear_speed}, angular={angular_speed}")
        self.twist_msg = self.create_twist_msg(linear_speed, angular_speed)
        self.direction_timer = self.create_timer(10.0, self.toggle_direction)
        
        self.is_moving = True
        self.publish_cmd(self.twist_msg)
        self.get_logger().info(f"Published initial movement command: linear={self.twist_msg.twist.linear.x}, angular={self.twist_msg.twist.angular.z}")

    def toggle_direction(self):
        """Alternate between sharp left and right turns"""
        if not self.is_moving:
            return
            
        # Alternate between sharp left and right turns
        if self.twist_msg.twist.angular.z > 0:
            self.twist_msg.twist.angular.z = -0.8
            self.get_logger().info(f"Changing direction to RIGHT (angular.z = -0.8)")
        else:
            self.twist_msg.twist.angular.z = 0.8
            self.get_logger().info(f"Changing direction to LEFT (angular.z = 0.8)")

    def analyze_slip(self):
        """Analyze slip using the slip detector"""
        if not self.is_moving:
            return
            
        odometry_distance = self.odometry_tracker.distance_traveled
        self.slip_detector.calculate_slip(odometry_distance)

    def stop_robot(self):
        """Send a zero velocity command to stop the robot"""
        self.get_logger().info("Stopping robot with zero velocity command")
        
        # Cancel direction timer if it exists
        if hasattr(self, 'direction_timer'):
            self.direction_timer.cancel()
        
        # Set the flag
        self.is_moving = False
        
        # Set velocity to zero
        self.twist_msg = self.create_twist_msg()
        
        # Send multiple stop commands to ensure it's received
        for _ in range(5):
            self.publisher_.publish(self.twist_msg)
            time.sleep(0.05)

    def _shutdown_callback(self, request, response):
        """Callback for shutdown service"""
        self.get_logger().info("Shutdown service called")
        self.stop_robot()
        return response

    def start_timed_run(self, duration=600.0):
        """Start a timed run for the specified duration in seconds"""
        self.run_duration = duration
        self.start_time = self.get_clock().now()
        self.get_logger().info(f"Starting timed run for {duration} seconds")
        
        # Create a timer to check if run duration has elapsed
        self.duration_timer = self.create_timer(1.0, self.check_duration)
        
    def check_duration(self):
        """Check if the run duration has elapsed"""
        if self.start_time is None:
            return
            
        current_time = self.get_clock().now()
        elapsed = (current_time - self.start_time).nanoseconds / 1e9
        
        # Log remaining time every 10 seconds
        if int(elapsed) % 10 == 0:
            remaining = max(0, self.run_duration - elapsed)
            self.get_logger().info(f"Time remaining: {remaining:.1f} seconds")
        
        if elapsed >= self.run_duration:
            self.get_logger().info(f"Timed run complete! Ran for {elapsed:.1f} seconds")
            self.get_logger().info(f"Final distance - Straight: {self.odometry_tracker.distance_traveled:.2f}m, Arc: {self.odometry_tracker.arc_distance_traveled:.2f}m")
            self.get_logger().info(
                f"Wheel-based distance: {self.slip_detector.wheel_traveled_distance:.2f}m, "
                f"Slip ratio: {self.slip_detector.slip_ratio:.2f}"
            )
            
            self.duration_timer.cancel()
            self.stop_robot()
            
            self.get_logger().info("Initiating node shutdown...")
            rclpy.shutdown()

    def create_twist_msg(self, linear_speed=0.0, angular_speed=0.0):
        """Create a TwistStamped message with current timestamp"""
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = linear_speed
        msg.twist.angular.z = angular_speed
        return msg

def main():
    rclpy.init()
    test_node = AckermannBotDemoTest()
    
    # Setup signal handler
    def signal_handler(sig, frame):
        print("Signal received, stopping robot...")
        test_node.stop_robot()
        test_node.destroy_node()
        rclpy.shutdown()
        sys.exit(0)
        
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
        
    # Start timed run for 2 minutes
    test_node.start_timed_run(duration=120.0)
    
    # Start movement after a brief delay
    time.sleep(2.0)  # Wait 2 seconds before starting movement
    test_node.start_moving(linear_speed=1.5, angular_speed=0.8)
    
    try:
        # Spin the node to process callbacks
        rclpy.spin(test_node)
    except KeyboardInterrupt:
        print("User interrupted execution, stopping robot...")
    finally:
        # Stop the robot before shutting down
        test_node.stop_robot()
        # Clean up
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()