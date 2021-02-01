#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from task_planner.msg import Action
from task_planner.msg import Monitor


def talker():
    pub = rospy.Publisher('action', Action, queue_size=10)
    rospy.init_node('tamp_interface', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    msg = Action()
    mon = Monitor()
    mon.predicate = "collision_free"
    mon.arguments = ["left_arm", "obstacle"]
    msg.id = "move_above_left_arm_slot11_slot21"
    msg.monitors = [mon]
    msg.succeed = False

    while not rospy.is_shutdown():
        #rospy.loginfo(msg)
        pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
