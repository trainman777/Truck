#!/usr/bin/env python
#
# This subscribes to a cmd_vel and publishes an odom with only 
# the vx in it.

import rospy
import tf
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
roll = pitch = yaw = 0.0

def callback(data):
    print "Vx=" + str(data.linear.x)
    pub = rospy.Publisher('odom', Odometry, queue_size=1)
    o = Odometry()
    o.child_frame_id = "odom"
    o.twist.twist = data
    o.header.frame_id = "base_link"
    o.twist.covariance[0] = 0.1
    pub.publish(o)    

def listener():
    rospy.init_node('odom_sim', anonymous=True)
    rospy.Subscriber("cmd_vel", Twist, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
