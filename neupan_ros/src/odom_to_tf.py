#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
import tf

class OdomToTFBroadcaster(object):
    def __init__(self):
        self.odom_topic = rospy.get_param('~odom_topic', '/wheel_odometry_raw')
        self.odom_frame = rospy.get_param('~odom_frame', 'odom')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.br = tf.TransformBroadcaster()
        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb, queue_size=10)

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.br.sendTransform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w), msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now(), self.base_frame, self.odom_frame)

if __name__ == '__main__':
    rospy.init_node('odom_to_tf_broadcaster')
    node = OdomToTFBroadcaster()
    rospy.spin() 