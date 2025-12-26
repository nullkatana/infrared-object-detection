#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import json
from collections import defaultdict


class KalmanFilter:
    """
    Simple Kalman Filter for 3D position tracking with constant velocity model.
    State: [x, y, z, vx, vy, vz]
    """
    
    def __init__(self, initial_position, dt=0.5):
        # State: [x, y, z, vx, vy, vz]
        self.state = np.array([
            initial_position[0],
            initial_position[1],
            initial_position[2],
            0.0, 0.0, 0.0  # Initial velocities
        ])
        
        # Time step
        self.dt = dt
        
        # State transition matrix (constant velocity model)
        self.F = np.array([
            [1, 0, 0, dt, 0, 0],
            [0, 1, 0, 0, dt, 0],
            [0, 0, 1, 0, 0, dt],
            [0, 0, 0, 1, 0, 0],
            [0, 0, 0, 0, 1, 0],
            [0, 0, 0, 0, 0, 1]
        ])
        
        # Measurement matrix (we only measure position)
        self.H = np.array([
            [1, 0, 0, 0, 0, 0],
            [0, 1, 0, 0, 0, 0],
            [0, 0, 1, 0, 0, 0]
        ])
        
        # Process noise covariance
        q = 0.1  # Process noise
        self.Q = np.eye(6) * q
        
        # Measurement noise covariance
        r = 0.05  # Measurement noise
        self.R = np.eye(3) * r
        
        # State covariance matrix
        self.P = np.eye(6) * 1.0
        
    def predict(self):
        """Predict next state"""
        self.state = self.F @ self.state
        self.P = self.F @ self.P @ self.F.T + self.Q
        return self.state[:3]  # Return predicted position
    
    def update(self, measurement):
        """Update state with new measurement"""
        # Innovation
        y = measurement - self.H @ self.state
        
        # Innovation covariance
        S = self.H @ self.P @ self.H.T + self.R
        
        # Kalman gain
        K = self.P @ self.H.T @ np.linalg.inv(S)
        
        # Update state
        self.state = self.state + K @ y
        
        # Update covariance
        self.P = (np.eye(6) - K @ self.H) @ self.P
        
        return self.state[:3], self.state[3:6]  # Return position and velocity
    
    def get_position(self):
        """Get current position estimate"""
        return self.state[:3]
    
    def get_velocity(self):
        """Get current velocity estimate"""
        return self.state[3:6]


class Track:
    """Represents a tracked object"""
    
    def __init__(self, track_id, initial_detection, dt=0.5):
        self.id = track_id
        self.kalman = KalmanFilter(initial_detection['position'], dt)
        self.last_detection = initial_detection
        self.missed_frames = 0
        self.hits = 1
        self.age = 0
        self.trajectory = [initial_detection['position']]
        
    def predict(self):
        """Predict next position"""
        return self.kalman.predict()
    
    def update(self, detection):
        """Update track with new detection"""
        position = np.array(detection['position'])
        pos, vel = self.kalman.update(position)
        
        self.last_detection = detection
        self.missed_frames = 0
        self.hits += 1
        self.trajectory.append(pos.tolist())
        
        return pos, vel
    
    def mark_missed(self):
        """Mark this track as having no detection this frame"""
        self.missed_frames += 1
        self.age += 1


class ObjectTracker(Node):
    """
    Multi-object tracker using Kalman filtering.
    Maintains tracks across frames with data association.
    """
    
    def __init__(self):
        super().__init__('object_tracker')
        
        # Subscribe to detections
        self.detection_subscription = self.create_subscription(
            String,
            'detections/objects',
            self.detection_callback,
            10
        )
        
        # Publisher for tracked objects
        self.tracking_publisher = self.create_publisher(
            String,
            'tracking/objects',
            10
        )
        
        # Tracking parameters
        self.max_missed_frames = 5  # Delete track after this many missed frames
        self.min_hits = 3  # Minimum hits before track is confirmed
        self.association_threshold = 0.5  # Maximum distance (m) to associate detection to track
        
        # Active tracks
        self.tracks = {}  # track_id -> Track
        self.next_track_id = 1
        
        # Timer for publishing tracking results
        self.timer = self.create_timer(0.5, self.publish_tracks)
        
        self.get_logger().info('Object Tracker started!')
        self.get_logger().info(f'Association threshold: {self.association_threshold}m')
        self.get_logger().info(f'Max missed frames: {self.max_missed_frames}')
        
    def detection_callback(self, msg):
        """Process incoming detections and update tracks"""
        try:
            data = json.loads(msg.data)
            detections = data.get('objects', [])
            
            if len(detections) == 0:
                # No detections - mark all tracks as missed
                for track in self.tracks.values():
                    track.mark_missed()
                return
            
            # Predict all tracks
            predictions = {}
            for track_id, track in self.tracks.items():
                predictions[track_id] = track.predict()
            
            # Data association: match detections to tracks
            matched_tracks, unmatched_detections = self.associate_detections(
                detections, predictions
            )
            
            # Update matched tracks
            for track_id, detection in matched_tracks.items():
                pos, vel = self.tracks[track_id].update(detection)
                speed = np.linalg.norm(vel)
                
                self.get_logger().info(
                    f"Track #{track_id}: pos=[{pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}], "
                    f"speed={speed:.2f}m/s"
                )
            
            # Mark unmatched tracks as missed
            matched_ids = set(matched_tracks.keys())
            for track_id in self.tracks.keys():
                if track_id not in matched_ids:
                    self.tracks[track_id].mark_missed()
            
            # Create new tracks for unmatched detections
            for detection in unmatched_detections:
                self.create_track(detection)
            
            # Delete dead tracks
            self.prune_tracks()
            
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Failed to parse detection data: {e}')
    
    def associate_detections(self, detections, predictions):
        """
        Associate detections with existing tracks using nearest neighbor.
        Returns: (matched_tracks, unmatched_detections)
        """
        matched = {}
        unmatched = []
        
        used_track_ids = set()
        
        for detection in detections:
            det_pos = np.array(detection['position'])
            
            # Find closest track
            best_track_id = None
            best_distance = self.association_threshold
            
            for track_id, pred_pos in predictions.items():
                if track_id in used_track_ids:
                    continue
                
                distance = np.linalg.norm(det_pos - pred_pos)
                
                if distance < best_distance:
                    best_distance = distance
                    best_track_id = track_id
            
            if best_track_id is not None:
                matched[best_track_id] = detection
                used_track_ids.add(best_track_id)
            else:
                unmatched.append(detection)
        
        return matched, unmatched
    
    def create_track(self, detection):
        """Create a new track from detection"""
        track_id = self.next_track_id
        self.tracks[track_id] = Track(track_id, detection)
        self.next_track_id += 1
        
        self.get_logger().info(
            f"Created track #{track_id} at position {detection['position']}"
        )
    
    def prune_tracks(self):
        """Remove tracks that haven't been seen recently"""
        to_delete = []
        
        for track_id, track in self.tracks.items():
            if track.missed_frames > self.max_missed_frames:
                to_delete.append(track_id)
        
        for track_id in to_delete:
            self.get_logger().info(f"Deleted track #{track_id} (lost)")
            del self.tracks[track_id]
    
    def publish_tracks(self):
        """Publish current tracking state"""
        confirmed_tracks = []
        
        for track_id, track in self.tracks.items():
            # Only publish confirmed tracks
            if track.hits >= self.min_hits:
                pos = track.kalman.get_position()
                vel = track.kalman.get_velocity()
                
                confirmed_tracks.append({
                    'track_id': track_id,
                    'position': pos.tolist(),
                    'velocity': vel.tolist(),
                    'speed': float(np.linalg.norm(vel)),
                    'hits': track.hits,
                    'age': track.age,
                    'trajectory_length': len(track.trajectory)
                })
        
        # Create message
        tracking_data = {
            'timestamp': self.get_clock().now().to_msg().sec,
            'num_tracks': len(confirmed_tracks),
            'tracks': confirmed_tracks
        }
        
        msg = String()
        msg.data = json.dumps(tracking_data, indent=2)
        self.tracking_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()