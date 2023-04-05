#!/usr/bin/env python
import rospy
from std_msgs.msg import String


def main():
    rospy.init_node("publisher")

    pub = rospy.Publisher("chatter", String, queue_size=10)

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        msg = "hello world {}".format(rospy.get_time())
        pub.publish(msg)

        rospy.loginfo("Message '{}' published".format(msg))

        rate.sleep()


if __name__ == "__main__":
    main()