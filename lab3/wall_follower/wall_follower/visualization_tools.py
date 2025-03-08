#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class VisualizationTools:
    """
    Class containing useful visualization tools for the wall follower
    """
    
    @staticmethod
    def make_line_marker(points, frame_id, marker_id=0, color=None):
        """
        Creates a line marker for visualization in rviz
        
        Args:
            points: List of points (geometry_msgs.msg.Point) for the line
            frame_id: Frame ID for the marker
            marker_id: ID for the marker
            color: Color for the marker (std_msgs.msg.ColorRGBA)
            
        Returns:
            visualization_msgs.msg.Marker: Line marker
        """
        if color is None:
            color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
            
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.points = points
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Line width
        marker.color = color
        
        return marker
    
    @staticmethod
    def make_point_marker(point, frame_id, marker_id=0, color=None, scale=0.2):
        """
        Creates a point marker for visualization in rviz
        
        Args:
            point: Point (geometry_msgs.msg.Point) for the marker
            frame_id: Frame ID for the marker
            marker_id: ID for the marker
            color: Color for the marker (std_msgs.msg.ColorRGBA)
            scale: Size of the point
            
        Returns:
            visualization_msgs.msg.Marker: Point marker
        """
        if color is None:
            color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position = point
        marker.pose.orientation.w = 1.0
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale
        marker.color = color
        
        return marker
