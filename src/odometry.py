#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TwistWithCovarianceStamped

from robomaster import robot 
import numpy as np

def quaternion_from_euler(roll, pitch, yaw):
	"""
	Convert an Euler angle to a quaternion.

	Input
	:param roll: The roll (rotation around x-axis) angle in radians.
	:param pitch: The pitch (rotation around y-axis) angle in radians.
	:param yaw: The yaw (rotation around z-axis) angle in radians.

	Output
	:return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
	"""
	qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
	qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
	qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
	qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

	return [qx, qy, qz, qw]

class OdometryPublisher:
    def __init__(self):

        self.ep_robot = robot.Robot()
        self.ep_robot.initialize(conn_type="rndis")
        
        self.ep_chassis = self.ep_robot.chassis

        self.ep_chassis.sub_position(freq=10, callback=self.update_pose)
        self.ep_chassis.sub_attitude(freq=10, callback=self.update_angle)


        rospy.init_node('odometry_publisher', anonymous=True)
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.rate = rospy.Rate(10) # 10 Hz
        self.theta = 0.0
        self.y = 0.0
        self.x = 0.0

        self.x_velocity = 0.0
        self.y_velocity = 0.0

        self.angular_z = 0.0

        self.prev_time = rospy.get_rostime().to_sec()

        self.prev_x = 0.0
        self.prev_y = 0.0
        self.prev_theta = 0.0


    def publish_odometry(self):
        odom = Odometry()
        q = quaternion_from_euler(0, 0, self.theta)

        odom.header.stamp = rospy.get_rostime()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = self.x_velocity
        odom.twist.twist.linear.y=self.y_velocity
        odom.twist.twist.angular.z = self.angular_z
        odom.pose.covariance = [0.1, 0, 0, 0, 0, 0,
                                0, 0.1, 0, 0, 0, 0,
                                0, 0, 0.1, 0, 0, 0,
                                0, 0, 0, 0.1, 0, 0,
                                0, 0, 0, 0, 0.1, 0,
                                0, 0, 0, 0, 0, 0.1] 
        
       
        self.odom_pub.publish(odom)
        self.rate.sleep()
        self.update_velocity()

    def update_angle(self, angle):
        yaw, _, _ = angle
        self.theta = np.deg2rad(yaw)

    def update_pose(self, pose):
        x, y, _ = pose
        self.x = x
        self.y = y

    def update_velocity(self):

        current_time = rospy.get_rostime().to_sec()
        dt = current_time - self.prev_time
        self.prev_time = current_time

        self.x_velocity = (self.x - self.prev_x) / dt
        self.y_velocity = (self.y - self.prev_y) / dt
        self.angular_z = (self.theta - self.prev_theta) / dt

        self.prev_x = self.x
        self.prev_y = self.y
        self.prev_theta = self.theta



    def spin(self):
        while not rospy.is_shutdown():
            self.publish_odometry()


if __name__ == '__main__':
    try:
        odometry_publisher = OdometryPublisher()
        odometry_publisher.spin()
    except rospy.ROSInterruptException:
        pass

