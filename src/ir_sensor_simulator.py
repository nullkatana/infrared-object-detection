#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from std_msgs.msg import String
import numpy as np
import random
import json


class IRSensorSimulator(Node):
    """
    Simulates an infrared distance sensor that detects objects.
    Publishes Range messages with simulated distance measurements.
    """
    
    def __init__(self):
        super().__init__('ir_sensor_simulator')
        
        # Create publisher for IR sensor data
        self.publisher = self.create_publisher(Range, 'ir_sensor/range', 10)
        
        # Timer to publish sensor data at 10 Hz
        self.timer = self.create_timer(0.1, self.publish_sensor_data)

        # Subscribe to shared object state
        self.state_subscription = self.create_subscription(
            String,
            'object/state',
            self.state_callback,
            10
        )
        
        # Simulation parameters
        self.min_range = 0.1  # meters
        self.max_range = 3.0  # meters
        self.fov = 0.1  # field of view in radians (~5.7 degrees)
        
        # Simulated object position (distance from sensor)
        self.object_distance = 1.5  # meters
        self.object_present = True
        
        self.get_logger().info('IR Sensor Simulator started!')
        self.get_logger().info(f'Range: {self.min_range}m to {self.max_range}m')
        
    def publish_sensor_data(self):
        """Publish simulated IR sensor reading"""
        msg = Range()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'ir_sensor_frame'
        
        # Sensor characteristics
        msg.radiation_type = Range.INFRARED
        msg.field_of_view = self.fov
        msg.min_range = self.min_range
        msg.max_range = self.max_range
        
        # Simulate distance reading with some noise
        if self.object_present:
            # Add Gaussian noise to simulate real sensor
            noise = np.random.normal(0, 0.02)  # 2cm standard deviation
            measured_distance = self.object_distance + noise
            
            # Clamp to valid range
            measured_distance = max(self.min_range, min(self.max_range, measured_distance))
            msg.range = measured_distance
        else:
            # No object detected - return max range
            msg.range = self.max_range
        
        # Publish the message
        self.publisher.publish(msg)
        
    def state_callback(self, msg):
        """Receive shared object state"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            if self.object_present:
                # Calculate distance from sensor (at origin) to object
                position = np.array(state['position'])
                self.object_distance = position[0]  # X-distance (sensor points along X)
        except json.JSONDecodeError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = IRSensorSimulator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()