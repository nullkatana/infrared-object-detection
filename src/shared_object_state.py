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
        self.object_velocity = np.array([0.0, 0.0, 0.0])  # Current velocity
        self.object_size = 0.5  # meters
        self.object_present = True
        
        # Movement parameters
        self.min_distance = 0.5
        self.max_distance = 2.5
        self.max_speed = 0.3  # Maximum speed in m/s
        self.change_direction_probability = 0.1  # 10% chance to change direction per second
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
        
        # Update object movement if present
        if self.object_present:
            # Randomly change direction
            if np.random.random() < self.change_direction_probability:
                # Generate new random velocity
                self.object_velocity = np.array([
                    np.random.uniform(-self.max_speed, self.max_speed),
                    np.random.uniform(-self.max_speed, self.max_speed),
                    0.0  # Keep Z velocity at 0 (no vertical movement)
                ])
                
                speed = np.linalg.norm(self.object_velocity)
                if speed > 0.01:
                    self.get_logger().info(
                        f'Object velocity changed: speed={speed:.2f}m/s'
                    )
            
            # Update position based on velocity
            dt = 1.0  # Time step (1 second)
            self.object_position += self.object_velocity * dt
            
            # Clamp position to valid range
            self.object_position[0] = np.clip(self.object_position[0], self.min_distance, self.max_distance)
            self.object_position[1] = np.clip(self.object_position[1], -1.5, 1.5)
            self.object_position[2] = np.clip(self.object_position[2], 0.3, 0.7)
            
            # Bounce off boundaries (reverse velocity if hit wall)
            if self.object_position[0] <= self.min_distance or self.object_position[0] >= self.max_distance:
                self.object_velocity[0] *= -1
            if self.object_position[1] <= -1.5 or self.object_position[1] >= 1.5:
                self.object_velocity[1] *= -1
        
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