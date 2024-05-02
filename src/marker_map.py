#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

class MarkerTf:
    def __init__(self):
        rospy.init_node('marker_tf_listener')
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        self.subscriber = rospy.Subscriber('marker_locations', PointStamped, self.callback)
        self.publisher = rospy.Publisher('marker_markers', Marker, queue_size=10)

    def callback(self, point_msg):
        try:
            # Get the transform from base_link to map
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(1.0))
            
            # Transform the point from base_link to map frame
            point_transformed = tf2_geometry_msgs.do_transform_point(point_msg, transform)
            
            # Publish the marker at the transformed point
            self.publish_marker(point_transformed)
        
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn("TF transform error: %s", e)

    def publish_marker(self, point_transformed):
        message = Marker()
        message.header = point_transformed.header
        message.type = Marker.SPHERE
        message.action = Marker.ADD
        message.pose.position = point_transformed.point
        message.pose.orientation.w = 1.0
        message.scale.x, message.scale.y, message.scale.z = 0.2, 0.2, 0.2
        message.color.a, message.color.r, message.color.g, message.color.b = 1.0, 1.0, 0.0, 0.0
        
        self.publisher.publish(message)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    marker_tf_listener = MarkerTf()
    marker_tf_listener.run()

