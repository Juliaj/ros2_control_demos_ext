#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import argparse
import os
from datetime import datetime


class OdomRecorder(Node):
    def __init__(self, output_file):
        super().__init__('odom_recorder')
        self.output_file = output_file
        self.subscription = self.create_subscription(
            Odometry,
            '/ackermann_steering_controller/odom',
            self.odom_callback,
            10)
        
        self.data = []
        self.start_time = None
        self.get_logger().info(f'Recording odometry data to {output_file}')

    def odom_callback(self, msg):
        if self.start_time is None:
            self.start_time = self.get_clock().now().seconds_nanoseconds()[0]
        
        current_time = self.get_clock().now().seconds_nanoseconds()[0] - self.start_time
        
        # Extract data from odometry message
        linear_x = msg.twist.twist.linear.x
        linear_y = msg.twist.twist.linear.y
        angular_z = msg.twist.twist.angular.z
        pos_x = msg.pose.pose.position.x
        pos_y = msg.pose.pose.position.y
        
        # Store data
        self.data.append({
            'time': current_time,
            'linear_x': linear_x,
            'linear_y': linear_y,
            'angular_z': angular_z,
            'pos_x': pos_x,
            'pos_y': pos_y
        })
        
        if len(self.data) % 100 == 0:
            self.get_logger().info(f'Recorded {len(self.data)} odometry points')

    def save_data(self):
        if not self.data:
            self.get_logger().error('No data recorded!')
            return
        
        df = pd.DataFrame(self.data)
        df.to_csv(self.output_file, index=False)
        self.get_logger().info(f'Saved {len(self.data)} odometry points to {self.output_file}')


def compare_odom_data(file1, file2, output_dir):
    """Compare two odometry data files and generate comparison plots"""
    
    # Load data
    df1 = pd.read_csv(file1)
    df2 = pd.read_csv(file2)
    
    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)
    
    # Create comparison plots
    # 1. Trajectory comparison
    plt.figure(figsize=(10, 8))
    plt.plot(df1['pos_x'], df1['pos_y'], 'b-', label='Run 1')
    plt.plot(df2['pos_x'], df2['pos_y'], 'r-', label='Run 2')
    plt.title('Trajectory Comparison')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, 'trajectory_comparison.png'))
    
    # 2. Linear velocity comparison
    plt.figure(figsize=(12, 6))
    plt.plot(df1['time'], df1['linear_x'], 'b-', label='Run 1')
    plt.plot(df2['time'], df2['linear_x'], 'r-', label='Run 2')
    plt.title('Linear Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity X (m/s)')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, 'linear_velocity_comparison.png'))
    
    # 3. Angular velocity comparison
    plt.figure(figsize=(12, 6))
    plt.plot(df1['time'], df1['angular_z'], 'b-', label='Run 1')
    plt.plot(df2['time'], df2['angular_z'], 'r-', label='Run 2')
    plt.title('Angular Velocity Comparison')
    plt.xlabel('Time (s)')
    plt.ylabel('Angular Velocity Z (rad/s)')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, 'angular_velocity_comparison.png'))
    
    # 4. Lateral velocity comparison (indicator of slipping)
    plt.figure(figsize=(12, 6))
    plt.plot(df1['time'], df1['linear_y'], 'b-', label='Run 1')
    plt.plot(df2['time'], df2['linear_y'], 'r-', label='Run 2')
    plt.title('Lateral Velocity Comparison (Slipping Indicator)')
    plt.xlabel('Time (s)')
    plt.ylabel('Linear Velocity Y (m/s)')
    plt.legend()
    plt.grid(True)
    plt.savefig(os.path.join(output_dir, 'lateral_velocity_comparison.png'))
    
    # Calculate slippage metrics
    slip_metric1 = np.abs(df1['linear_y']).mean()
    slip_metric2 = np.abs(df2['linear_y']).mean()
    slip_difference = ((slip_metric2 - slip_metric1) / slip_metric1) * 100 if slip_metric1 > 0 else float('inf')
    
    # Generate summary report
    with open(os.path.join(output_dir, 'comparison_report.txt'), 'w') as f:
        f.write("Odometry Comparison Report\n")
        f.write("=========================\n\n")
        f.write(f"Run 1: {file1}\n")
        f.write(f"Run 2: {file2}\n\n")
        f.write(f"Total distance Run 1: {calculate_distance(df1):.2f} meters\n")
        f.write(f"Total distance Run 2: {calculate_distance(df2):.2f} meters\n\n")
        f.write(f"Average linear velocity Run 1: {df1['linear_x'].mean():.3f} m/s\n")
        f.write(f"Average linear velocity Run 2: {df2['linear_x'].mean():.3f} m/s\n\n")
        f.write(f"Average lateral velocity (slipping) Run 1: {slip_metric1:.5f} m/s\n")
        f.write(f"Average lateral velocity (slipping) Run 2: {slip_metric2:.5f} m/s\n")
        f.write(f"Slippage increase: {slip_difference:.2f}%\n\n")
        f.write("Graphs generated in the same directory.\n")
    
    print(f"Comparison complete. Results saved to {output_dir}")


def calculate_distance(df):
    """Calculate total distance traveled from position data"""
    dx = df['pos_x'].diff().fillna(0)
    dy = df['pos_y'].diff().fillna(0)
    distances = np.sqrt(dx**2 + dy**2)
    return distances.sum()


def main():
    parser = argparse.ArgumentParser(description='Record and compare odometry data')
    parser.add_argument('--record', action='store_true', help='Record odometry data')
    parser.add_argument('--compare', action='store_true', help='Compare two odometry data files')
    parser.add_argument('--output', type=str, help='Output file for recording', default=None)
    parser.add_argument('--file1', type=str, help='First file for comparison')
    parser.add_argument('--file2', type=str, help='Second file for comparison')
    parser.add_argument('--output-dir', type=str, help='Output directory for comparison results', default='comparison_results')
    
    args = parser.parse_args()
    
    if args.record:
        # Record mode
        if args.output is None:
            timestamp = datetime.now().strftime("%Y%m%d-%H%M%S")
            args.output = f"odom_data_{timestamp}.csv"
        
        rclpy.init()
        recorder = OdomRecorder(args.output)
        
        try:
            rclpy.spin(recorder)
        except KeyboardInterrupt:
            pass
        finally:
            recorder.save_data()
            recorder.destroy_node()
            rclpy.shutdown()
            
    elif args.compare:
        # Compare mode
        if not args.file1 or not args.file2:
            print("Error: Must provide two files to compare with --file1 and --file2")
            return
        
        compare_odom_data(args.file1, args.file2, args.output_dir)
    
    else:
        print("Please specify either --record or --compare")


if __name__ == '__main__':
    main() 