#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from task_planner.msg import Action
from task_planner.msg import Monitor

pub = rospy.Publisher('action_status', action, queue_size=10)
msg = Action()
collision_monitor = Monitor()
admittance_monitor = Monitor()
last_action_left_arm = "none"
last_action_right_arm = "none"

def callback_action(data):
    #rospy.loginfo("%s " % data.data)
    # parse data
    str = data.data[2:-10]
    msg.id = str
    collision_monitor.predicate = "collision_free"
    collision_monitor.arguments = ["left_arm", "obstacle"]
    admittance_monitor.predicate = "admittance_free"
    admittance_monitor.arguments = ["left_arm", "obstacle"]
    msg.monitors  = [collision_monitor, admittance_monitor]
    msg.succeed   = True
    pub.publish(msg)

def callback_collision(data):
    print("here")

def callback_admittance(data):
    print("here")

def talker():
    mon.predicate = "collision_free"
    mon.arguments = ["left_arm", "obstacle"]
    msg.id = "move_above_left_arm_slot11_slot21"
    msg.monitors = [mon]
    msg.succeed = False
    
    
    pub = rospy.Publisher('action', Action, queue_size=10)
    rospy.Subscriber("eventPort", String, callback_action)    
    rospy.Subscriber("collision_status", String, callback_collision)    
    rospy.Subscriber("callback_admittance", String, callback_admittance)    
    rospy.init_node('action_monitor_publisher', anonymous=True)
    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        #rospy.loginfo(msg)
        pub.publish(msg)
        rospy.sleep(0.1)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
