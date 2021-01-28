#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from tamp_msgs.msg import action
from tamp_msgs.msg import monitor


def talker():
    pub = rospy.Publisher('chatter', action, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    msg = action()
    mon = monitor()
    mon.predicate = "collision_free"
    mon.arguments = ["left_arm", "obstacle"]
    msg.id = "move_above"
    msg.monitors = [mon]
    msg.succeed = True
    while not rospy.is_shutdown():
        #rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
