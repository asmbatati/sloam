#!/usr/bin/env python

import rospy
import tf
import tf2_ros
import numpy as np
import math
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class ForestRobotInputManager:
    def __init__(self):
        rospy.init_node('forest_robot_input_manager', anonymous=True)
        
        # Load parameters
        self.min_odom_distance = rospy.get_param('~min_odom_distance', 0.5)
        self.cloud_topic = rospy.get_param('~cloud_topic', '/forest_robot/velodyne_points')
        self.odom_topic = rospy.get_param('~odom_topic', '/odom')
        self.output_cloud_topic = rospy.get_param('~output_cloud_topic', '/sloam/input_cloud')
        
        # Frame IDs
        self.map_frame = rospy.get_param('~map_frame', 'map')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.robot_frame = rospy.get_param('~robot_frame', 'base_link')
        
        # Initialize variables
        self.last_pose = None
        self.latest_cloud = None
        self.latest_odom = None
        self.total_distance = 0.0
        self.tf_listener = tf.TransformListener()
        
        # Publishers
        self.cloud_pub = rospy.Publisher(self.output_cloud_topic, PointCloud2, queue_size=10)
        self.pose_pub = rospy.Publisher('/sloam/input_pose', PoseStamped, queue_size=10)
        
        # Subscribers
        rospy.Subscriber(self.cloud_topic, PointCloud2, self.cloud_callback)
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback)
        
        rospy.loginfo("Forest Robot Input Manager initialized")
        
    def cloud_callback(self, cloud_msg):
        self.latest_cloud = cloud_msg
        self.process_data()
        
    def odom_callback(self, odom_msg):
        self.latest_odom = odom_msg
        self.process_data()
        
    def process_data(self):
        # Make sure we have both cloud and odometry data
        if self.latest_cloud is None or self.latest_odom is None:
            return
            
        # Get current position
        current_x = self.latest_odom.pose.pose.position.x
        current_y = self.latest_odom.pose.pose.position.y
        current_z = self.latest_odom.pose.pose.position.z
        
        # Initialize last_pose if this is the first callback
        if self.last_pose is None:
            self.last_pose = (current_x, current_y, current_z)
            return
            
        # Calculate distance moved since last keyframe
        dx = current_x - self.last_pose[0]
        dy = current_y - self.last_pose[1]
        dz = current_z - self.last_pose[2]
        distance = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # If we've moved enough, process this as a new keyframe
        if distance >= self.min_odom_distance:
            rospy.loginfo(f"Keyframe triggered: moved {distance:.2f}m")
            
            # Update last pose
            self.last_pose = (current_x, current_y, current_z)
            self.total_distance += distance
            
            # We'll just pass the cloud through without transforming it
            # since we removed tf2_sensor_msgs dependency
            # Publish the point cloud
            self.cloud_pub.publish(self.latest_cloud)
            
            # Create and publish pose
            pose_stamped = PoseStamped()
            pose_stamped.header = Header()
            pose_stamped.header.stamp = self.latest_odom.header.stamp
            pose_stamped.header.frame_id = self.odom_frame
            pose_stamped.pose = self.latest_odom.pose.pose
            
            self.pose_pub.publish(pose_stamped)
            
            rospy.loginfo("Published cloud and pose for SLOAM")

if __name__ == '__main__':
    try:
        manager = ForestRobotInputManager()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass