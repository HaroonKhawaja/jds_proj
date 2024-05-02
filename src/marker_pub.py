#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PointStamped
import cv2
from robomaster import robot
from robomaster import vision
import numpy as np

markers = []

class MarkerInfo:
    def __init__(self, x, y, w, h, info):
        self._x = x
        self._y = y
        self._w = w
        self._h = h
        self._info = info

    @property
    def pt1(self):
        return (int(self._x - self._w / 2), int(self._y - self._h / 2))

    @property
    def pt2(self):
        return (int(self._x + self._w / 2), int(self._y + self._h / 2))

    @property
    def center(self):
        return (int(self._x * 1280), int(self._y * 720))

    @property
    def text(self):
        return self._info

def distance(wp):
    if wp == 0:
        return 0
    else:
        return (0.17 * 630) / (wp * 1280)

def getAngle(p):
    pix = 120 / 1280
    mid_x = 1280 / 2
    diff_x = p[0] - mid_x
    return round(diff_x * pix, 2)

def getXY(p, distance):
    angles = getAngle(p)
    y = distance * np.sin(np.deg2rad(angles))
    return (round(y, 2), angles)

def on_detect_marker(marker_info):
    number = len(marker_info)
    markers.clear()
    for i in range(0, number):
        x, y, w, h, info = marker_info[i]
        markers.append(MarkerInfo(x, y, w, h, info))

def pub_location(x, y, z=0):
    point_msg = PointStamped()
    point_msg.header.frame_id = "base_link"
    point_msg.header.stamp = rospy.Time.now()
    point_msg.point.x = x
    point_msg.point.y = y
    point_msg.point.z = z
    position_pub.publish(point_msg)

rospy.init_node('marker_publisher')
position_pub = rospy.Publisher('marker_positions', PointStamped, queue_size=10)

if __name__ == '__main__':
    ep_robot = robot.Robot()
    ep_robot.initialize(conn_type="rndis")
    ep_vision = ep_robot.vision
    ep_camera = ep_robot.camera
    ep_camera.start_video_stream(display=False)
    result = ep_vision.sub_detect_info(name="marker", callback=on_detect_marker)

    try:
        while not rospy.is_shutdown():
            img = ep_camera.read_cv2_image(strategy="newest", timeout=0.5)
            for marker in markers:
                x = round(distance(marker._w), 2)
                y, angle = getXY(marker.center, x)
                pub_location(x, y)

            cv2.imshow("Markers", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except rospy.ROSInterruptException:
        pass

