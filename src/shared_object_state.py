#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json


class SharedObjectState(Node):
    """
    Publishes shared object state for synchronized sensor simulation.
    Both IR sensor and point cloud generator subscribe to this to ensure
    they're sensing the same object.
    """
    
    def __init__(self):
        super().__init__('shared_object_state')
        
        # Publisher for object state
        self.state_publisher = self.create_publisher(String, 'object/state', 10)
        
        # Timer to update object state at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_object_state)
        
        # Object parameters
        self.object_position = np.array([1.5, 0.0, 0.5])  # [x, y, z] in meters
        self.object_size = 0.5  # meters
        self.object_present = True
        
        # Movement parameters
        self.min_distance = 0.5
        self.max_distance = 2.5
        self.move_probability = 0.15  # 15% chance to move per second
        self.disappear_probability = 0.05  # 5% chance to disappear
        
        self.get_logger().info('Shared Object State publisher started!')
        self.get_logger().info(f'Initial object position: {self.object_position}')
        
    def publish_object_state(self):
        """Publish current object state"""
        
        # Check for disappear/reappear FIRST (independent of movement)
        if np.random.random() < self.disappear_probability:
            self.object_present = not self.object_present
            if self.object_present:
                self.get_logger().info('Object appeared')
            else:
                self.get_logger().info('Object disappeared')
        
        # Randomly move object if present
        if self.object_present and np.random.random() < self.move_probability:
            # Move to new position
            self.object_position[0] = np.random.uniform(self.min_distance, self.max_distance)
            self.object_position[1] = np.random.uniform(-1.0, 1.0)
            self.object_position[2] = np.random.uniform(0.3, 0.7)
            
            self.get_logger().info(
                f'Object moved to: [{self.object_position[0]:.2f}, '
                f'{self.object_position[1]:.2f}, {self.object_position[2]:.2f}]'
            )
        
        # Create state message
        state = {
            'present': self.object_present,
            'position': self.object_position.tolist(),
            'size': self.object_size,
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        
        # Publish
        msg = String()
        msg.data = json.dumps(state)
        self.state_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SharedObjectState()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()