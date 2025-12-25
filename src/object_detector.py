#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range, PointCloud2
from std_msgs.msg import String
import numpy as np
import struct
import json
from sklearn.cluster import DBSCAN


class ObjectDetector(Node):
    """
    Combines IR sensor and point cloud data to detect objects.
    Subscribes to both sensor topics and publishes detection results.
    """
    
    def __init__(self):
        super().__init__('object_detector')
        
        # Subscribers
        self.ir_subscription = self.create_subscription(
            Range,
            'ir_sensor/range',
            self.ir_callback,
            10
        )
        
        self.pc_subscription = self.create_subscription(
            PointCloud2,
            'pointcloud/scene',
            self.pointcloud_callback,
            10
        )
        
        # Publisher for detection results
        self.detection_publisher = self.create_publisher(
            String,
            'detections/objects',
            10
        )
        
        # Store latest sensor data
        self.latest_ir_range = None
        self.latest_pointcloud = None
        
        # Detection parameters
        self.detection_threshold = 0.2  # meters - minimum object size
        self.ir_detection_range = 2.5  # meters - max distance to trust IR
        
        # DBSCAN clustering parameters
        self.dbscan_eps = 0.35  # Maximum distance between points in a cluster (meters)
        self.dbscan_min_samples = 8  # Minimum points to form a cluster
        
        # Timer for periodic detection
        self.detection_timer = self.create_timer(0.5, self.perform_detection)
        
        self.get_logger().info('Object Detector started!')
        self.get_logger().info('Waiting for sensor data...')
        
    def ir_callback(self, msg):
        """Receive IR sensor data"""
        self.latest_ir_range = msg.range
        
    def pointcloud_callback(self, msg):
        """Receive and parse point cloud data"""
        self.latest_pointcloud = self.parse_pointcloud(msg)
        
    def parse_pointcloud(self, pc_msg):
        """Convert PointCloud2 message to numpy array"""
        # Extract points from binary data
        points = []
        point_step = pc_msg.point_step
        
        for i in range(0, len(pc_msg.data), point_step):
            x, y, z = struct.unpack('fff', pc_msg.data[i:i+12])
            points.append([x, y, z])
        
        return np.array(points)
    
    def detect_objects_in_pointcloud(self, points):
        """DBSCAN-based object detection in point cloud"""
        if len(points) == 0:
            return []
        
        # Filter points that are above ground (z > 0.1m)
        elevated_points = points[points[:, 2] > 0.1]
        
        if len(elevated_points) < self.dbscan_min_samples:
            return []
        
        # Apply DBSCAN clustering
        try:
            clustering = DBSCAN(eps=self.dbscan_eps, min_samples=self.dbscan_min_samples)
            labels = clustering.fit_predict(elevated_points)
        except Exception as e:
            self.get_logger().warn(f'DBSCAN failed: {e}')
            return []
        
        # Find unique clusters (excluding noise labeled as -1)
        unique_labels = set(labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)  # Remove noise label
        
        detections = []
        
        for label in unique_labels:
            # Get points belonging to this cluster
            cluster_points = elevated_points[labels == label]
            
            # Calculate object properties
            centroid = np.mean(cluster_points, axis=0)
            size = np.max(cluster_points, axis=0) - np.min(cluster_points, axis=0)
            volume = size[0] * size[1] * size[2]
            
            # Only report if volume is above threshold
            if volume > self.detection_threshold:
                detections.append({
                    'cluster_id': int(label),
                    'position': centroid.tolist(),
                    'size': size.tolist(),
                    'volume': float(volume),
                    'num_points': len(cluster_points),
                    'distance': float(centroid[0])
                })
        
        return detections
    
    def perform_detection(self):
        """Main detection logic - combines all sensor data"""
        if self.latest_pointcloud is None:
            return
        
        # Detect objects using point cloud
        detections = self.detect_objects_in_pointcloud(self.latest_pointcloud)
        
        # Create detection report
        report = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'ir_range': float(self.latest_ir_range) if self.latest_ir_range else None,
            'num_detections': len(detections),
            'objects': detections
        }
        
        # Publish results
        msg = String()
        msg.data = json.dumps(report, indent=2)
        self.detection_publisher.publish(msg)
        
        # Log significant detections
        if len(detections) > 0:
            for i, obj in enumerate(detections):
                self.get_logger().info(
                    f"Object {i+1}: Distance={obj['distance']:.2f}m, "
                    f"Volume={obj['volume']:.3f}m³, Points={obj['num_points']}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()