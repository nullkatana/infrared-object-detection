#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import csv
import os
from datetime import datetime
from collections import deque


class MetricsLogger(Node):
    """
    Logs performance metrics from the object detection system.
    Tracks detection rate, latency, and saves statistics to CSV.
    """
    
    def __init__(self):
        super().__init__('metrics_logger')
        
        # Subscribe to detection results
        self.detection_subscription = self.create_subscription(
            String,
            'detections/objects',
            self.detection_callback,
            10
        )
        
        # Metrics storage
        self.detection_times = deque(maxlen=100)  # Last 100 detection timestamps
        self.detection_count = 0
        self.total_detections = 0
        self.start_time = self.get_clock().now()
        
        # CSV file setup
        self.csv_file = 'data/metrics_log.csv'
        self.ensure_data_folder()
        self.initialize_csv()
        
        # Timer for periodic metrics reporting
        self.metrics_timer = self.create_timer(5.0, self.report_metrics)
        
        self.get_logger().info('Metrics Logger started!')
        self.get_logger().info(f'Logging to: {self.csv_file}')
        
    def ensure_data_folder(self):
        """Create data folder if it doesn't exist"""
        os.makedirs('data', exist_ok=True)
        
    def initialize_csv(self):
        """Initialize CSV file with headers"""
        file_exists = os.path.isfile(self.csv_file)
        
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                # Write headers
                writer.writerow([
                    'timestamp',
                    'total_detections',
                    'detection_rate_hz',
                    'avg_objects_per_detection',
                    'uptime_seconds'
                ])
                self.get_logger().info('Created new metrics log file')
            else:
                self.get_logger().info('Appending to existing metrics log')
    
    def detection_callback(self, msg):
        """Process incoming detection messages"""
        try:
            # Parse JSON detection data
            detection_data = json.loads(msg.data)
            
            # Record timestamp
            current_time = self.get_clock().now()
            self.detection_times.append(current_time.nanoseconds / 1e9)  # Convert to seconds
            
            # Update counters
            num_objects = detection_data.get('num_detections', 0)
            if num_objects > 0:
                self.detection_count += 1
            
            self.total_detections += num_objects
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse detection data: {e}')
    
    def calculate_detection_rate(self):
        """Calculate detections per second over recent window"""
        if len(self.detection_times) < 2:
            return 0.0
        
        time_span = self.detection_times[-1] - self.detection_times[0]
        if time_span > 0:
            return len(self.detection_times) / time_span
        return 0.0
    
    def report_metrics(self):
        """Calculate and report current metrics"""
        current_time = self.get_clock().now()
        uptime = (current_time - self.start_time).nanoseconds / 1e9
        
        detection_rate = self.calculate_detection_rate()
        
        # Calculate average objects per detection
        avg_objects = 0.0
        if self.detection_count > 0:
            avg_objects = self.total_detections / self.detection_count
        
        # Log to console
        self.get_logger().info(
            f'Metrics - Rate: {detection_rate:.2f} Hz, '
            f'Total: {self.total_detections}, '
            f'Avg/detection: {avg_objects:.2f}, '
            f'Uptime: {uptime:.1f}s'
        )
        
        # Write to CSV
        with open(self.csv_file, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                datetime.now().isoformat(),
                self.total_detections,
                f'{detection_rate:.3f}',
                f'{avg_objects:.3f}',
                f'{uptime:.1f}'
            ])


def main(args=None):
    rclpy.init(args=args)
    node = MetricsLogger()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Shutting down metrics logger')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()