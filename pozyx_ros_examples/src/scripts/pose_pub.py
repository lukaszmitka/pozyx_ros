#!/usr/bin/env python
"""
ROS node that publishes the pose (position + quaternion) of the Pozyx

This is an example of how to combine sensor data and positioning into a single
channel output.

Quite overkill using _Pose, as this consists of 7 float64s, while the source
data comes from integers. Suggestions to replace this are quite welcomed.
"""

import pypozyx
import rospy
from geometry_msgs.msg import Point, Pose, Quaternion, PoseStamped
import std_msgs.msg

remote_id = None


def pozyx_pose_pub():
    pub = rospy.Publisher('pozyx_pose', Pose, queue_size=40)
    pubStamped = rospy.Publisher('pozyx_pose_stamped', PoseStamped, queue_size=40)
    rospy.init_node('pozyx_pose_node')
    h = std_msgs.msg.Header()
    h.frame_id = "pozyx"
    try:
        pozyx = pypozyx.PozyxSerial(pypozyx.get_serial_ports()[0].device)
    except:
        rospy.loginfo("Pozyx not connected")
        return
    while not rospy.is_shutdown():
        coords = pypozyx.Coordinates()
        quat = pypozyx.Quaternion()
        pozyx.doPositioning(coords, pypozyx.POZYX_3D, remote_id=remote_id)
        pozyx.getQuaternion(quat, remote_id=remote_id)
        rospy.loginfo("POS: %s, QUAT: %s" % (str(coords), str(quat)))
        h.stamp = rospy.Time.now()
        pub.publish(Point(coords.x, coords.y, coords.z), Quaternion(quat.x, quat.y, quat.z, quat.w))
        pubStamped.publish(h, Pose(Point(float(coords.x)/1000, float(coords.y)/1000, float(coords.z)/1000), Quaternion(quat.x, quat.y, quat.z, quat.w)))


if __name__ == '__main__':
    try:
        pozyx_pose_pub()
    except rospy.ROSInterruptException:
        pass
