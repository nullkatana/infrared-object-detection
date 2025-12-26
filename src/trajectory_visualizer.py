#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import numpy as np
import json


class TrajectoryVisualizer(Node):
    """
    Visualizes object trajectories and predictions in RViz2.
    Subscribes to tracking data and publishes visualization markers.
    """
    
    def __init__(self):
        super().__init__('trajectory_visualizer')
        
        # Subscribe to tracking data
        self.tracking_subscription = self.create_subscription(
            String,
            'tracking/objects',
            self.tracking_callback,
            10
        )
        
        # Publisher for visualization markers
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            'tracking/visualization',
            10
        )
        
        # Store trajectory history
        self.trajectories = {}  # track_id -> list of positions
        self.max_trajectory_length = 50  # Keep last 50 positions
        
        # Prediction parameters
        self.prediction_horizon = 3.0  # Predict 3 seconds ahead
        self.prediction_steps = 10  # Number of prediction points
        
        self.get_logger().info('Trajectory Visualizer started!')
        self.get_logger().info(f'Prediction horizon: {self.prediction_horizon}s')
        
    def tracking_callback(self, msg):
        """Process tracking data and create visualizations"""
        try:
            data = json.loads(msg.data)
            tracks = data.get('tracks', [])
            
            if len(tracks) == 0:
                # Clear all markers if no tracks
                self.publish_clear_markers()
                return
            
            # Create marker array
            marker_array = MarkerArray()
            marker_id = 0
            
            for track in tracks:
                track_id = track['track_id']
                position = np.array(track['position'])
                velocity = np.array(track['velocity'])
                
                # Update trajectory history
                if track_id not in self.trajectories:
                    self.trajectories[track_id] = []
                
                self.trajectories[track_id].append(position)
                
                # Keep only recent positions
                if len(self.trajectories[track_id]) > self.max_trajectory_length:
                    self.trajectories[track_id].pop(0)
                
                # Create markers for this track
                
                # 1. Current position marker (sphere)
                marker_id += 1
                marker_array.markers.append(
                    self.create_position_marker(track_id, position, marker_id)
                )
                
                # 2. Velocity arrow
                if np.linalg.norm(velocity) > 0.01:  # Only if moving
                    marker_id += 1
                    marker_array.markers.append(
                        self.create_velocity_arrow(track_id, position, velocity, marker_id)
                    )
                
                # 3. Trajectory history (line strip)
                if len(self.trajectories[track_id]) > 1:
                    marker_id += 1
                    marker_array.markers.append(
                        self.create_trajectory_line(track_id, self.trajectories[track_id], marker_id)
                    )
                
                # 4. Predicted trajectory
                predicted_positions = self.predict_trajectory(position, velocity)
                if len(predicted_positions) > 0:
                    marker_id += 1
                    marker_array.markers.append(
                        self.create_prediction_line(track_id, predicted_positions, marker_id)
                    )
                
                # 5. Track ID text
                marker_id += 1
                marker_array.markers.append(
                    self.create_text_marker(track_id, position, marker_id)
                )
            
            # Publish all markers
            self.marker_publisher.publish(marker_array)
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse tracking data: {e}')
    
    def predict_trajectory(self, position, velocity):
        """Predict future positions using constant velocity model"""
        predictions = []
        dt = self.prediction_horizon / self.prediction_steps
        
        for i in range(1, self.prediction_steps + 1):
            t = i * dt
            predicted_pos = position + velocity * t
            predictions.append(predicted_pos)
        
        return predictions
    
    def create_position_marker(self, track_id, position, marker_id):
        """Create sphere marker for current position"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'positions'
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2])
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        # Color based on track ID
        color = self.get_track_color(track_id)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 1.0
        
        marker.lifetime.sec = 1
        
        return marker
    
    def create_velocity_arrow(self, track_id, position, velocity, marker_id):
        """Create arrow marker showing velocity"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'velocities'
        marker.id = marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Arrow from current position to position + velocity
        start = Point()
        start.x = float(position[0])
        start.y = float(position[1])
        start.z = float(position[2])
        
        end = Point()
        end_pos = position + velocity * 0.5  # Scale for visibility
        end.x = float(end_pos[0])
        end.y = float(end_pos[1])
        end.z = float(end_pos[2])
        
        marker.points = [start, end]
        
        marker.scale.x = 0.05  # Shaft diameter
        marker.scale.y = 0.1   # Head diameter
        
        color = self.get_track_color(track_id)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.8
        
        marker.lifetime.sec = 1
        
        return marker
    
    def create_trajectory_line(self, track_id, trajectory, marker_id):
        """Create line strip for trajectory history"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'trajectories'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        for pos in trajectory:
            point = Point()
            point.x = float(pos[0])
            point.y = float(pos[1])
            point.z = float(pos[2])
            marker.points.append(point)
        
        marker.scale.x = 0.03  # Line width
        
        color = self.get_track_color(track_id)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.5
        
        marker.lifetime.sec = 1
        
        return marker
    
    def create_prediction_line(self, track_id, predictions, marker_id):
        """Create dashed line for predicted trajectory"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'predictions'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        for pos in predictions:
            point = Point()
            point.x = float(pos[0])
            point.y = float(pos[1])
            point.z = float(pos[2])
            marker.points.append(point)
        
        marker.scale.x = 0.02  # Thinner line
        
        color = self.get_track_color(track_id)
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = 0.3  # More transparent
        
        marker.lifetime.sec = 1
        
        return marker
    
    def create_text_marker(self, track_id, position, marker_id):
        """Create text marker showing track ID"""
        marker = Marker()
        marker.header.frame_id = 'world'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'labels'
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        marker.pose.position.x = float(position[0])
        marker.pose.position.y = float(position[1])
        marker.pose.position.z = float(position[2]) + 0.3  # Above object
        
        marker.text = f"Track #{track_id}"
        
        marker.scale.z = 0.15  # Text height
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.lifetime.sec = 1
        
        return marker
    
    def get_track_color(self, track_id):
        """Get consistent color for track ID"""
        colors = [
            (1.0, 0.0, 0.0),  # Red
            (0.0, 1.0, 0.0),  # Green
            (0.0, 0.0, 1.0),  # Blue
            (1.0, 1.0, 0.0),  # Yellow
            (1.0, 0.0, 1.0),  # Magenta
            (0.0, 1.0, 1.0),  # Cyan
        ]
        return colors[track_id % len(colors)]
    
    def publish_clear_markers(self):
        """Clear all markers"""
        marker_array = MarkerArray()
        
        # Delete all marker namespaces
        for ns in ['positions', 'velocities', 'trajectories', 'predictions', 'labels']:
            marker = Marker()
            marker.header.frame_id = 'world'
            marker.ns = ns
            marker.action = Marker.DELETEALL
            marker_array.markers.append(marker)
        
        self.marker_publisher.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()