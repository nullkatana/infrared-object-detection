#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String
import numpy as np
import struct
import json


class PointCloudGenerator(Node):
    """
    Generates simulated 3D point cloud data with objects.
    Publishes PointCloud2 messages representing a scene with detectable objects.
    """
    
    def __init__(self):
        super().__init__('pointcloud_generator')
        
        # Create publisher for point cloud data
        self.publisher = self.create_publisher(PointCloud2, 'pointcloud/scene', 10)
        
        # Timer to publish point cloud at 5 Hz
        self.timer = self.create_timer(0.2, self.publish_pointcloud)
        
        # Scene parameters
        self.scene_width = 5.0  # meters
        self.scene_depth = 5.0  # meters
        self.scene_height = 3.0  # meters
        
        # Object properties
        self.object_position = np.array([1.5, 0.0, 0.5])  # [x, y, z]
        self.object_size = 0.5  # meters

        # Subscribe to shared object state
        self.state_subscription = self.create_subscription(
            String,
            'object/state',
            self.state_callback,
            10
        )
        self.object_present = True
        
        self.get_logger().info('Point Cloud Generator started!')
        self.get_logger().info(f'Scene size: {self.scene_width}x{self.scene_depth}x{self.scene_height}m')
        
    def generate_ground_plane(self, num_points=500):
        """Generate points for the ground plane"""
        x = np.random.uniform(-self.scene_width/2, self.scene_width/2, num_points)
        y = np.random.uniform(-self.scene_depth/2, self.scene_depth/2, num_points)
        z = np.zeros(num_points)  # Ground at z=0
        
        return np.column_stack([x, y, z])
    
    def generate_object(self, center, size, num_points=200):
        """Generate points for a cubic object"""
        # Random points within a cube
        points = np.random.uniform(-size/2, size/2, (num_points, 3))
        
        # Translate to object position
        points += center
        
        return points
    
    def generate_noise_points(self, num_points=50):
        """Generate random noise points (simulating sensor noise)"""
        x = np.random.uniform(-self.scene_width/2, self.scene_width/2, num_points)
        y = np.random.uniform(-self.scene_depth/2, self.scene_depth/2, num_points)
        z = np.random.uniform(0, self.scene_height, num_points)
        
        return np.column_stack([x, y, z])
    
    def create_pointcloud2_msg(self, points):
        """Convert numpy array of points to PointCloud2 message"""
        msg = PointCloud2()
        
        # Header
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        
        # Point cloud dimensions
        msg.height = 1  # Unorganized point cloud
        msg.width = len(points)
        
        # Point fields (x, y, z)
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 12  # 3 floats * 4 bytes
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        
        # Pack point data
        buffer = []
        for point in points:
            buffer.append(struct.pack('fff', point[0], point[1], point[2]))
        
        msg.data = b''.join(buffer)
        
        return msg
    
    def publish_pointcloud(self):
        """Generate and publish a complete point cloud scene"""
        # Generate different components of the scene
        ground = self.generate_ground_plane(num_points=500)
        noise = self.generate_noise_points(num_points=50)
        
        # Only generate object if present
        if self.object_present:
            obj = self.generate_object(self.object_position, self.object_size, num_points=200)
            all_points = np.vstack([ground, obj, noise])
        else:
            all_points = np.vstack([ground, noise])
        
        # Create and publish message
        msg = self.create_pointcloud2_msg(all_points)
        self.publisher.publish(msg)

    def state_callback(self, msg):
        """Receive shared object state"""
        try:
            state = json.loads(msg.data)
            self.object_present = state['present']
            if self.object_present:
                self.object_position = np.array(state['position'])
        except json.JSONDecodeError:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudGenerator()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()