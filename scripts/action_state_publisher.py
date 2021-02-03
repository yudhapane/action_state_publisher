#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from task_planner.msg import Action
from task_planner.msg import Monitor

pub = rospy.Publisher('action_status', Action, queue_size=10)
msg = Action()
collision_monitor = Monitor()
admittance_monitor = Monitor()
last_action_right_arm = "none"
last_action_left_arm = "none"

def callback_action(data):
    #rospy.loginfo("%s " % data.data)
    # parse data
    str = data.data[2:-10]
    msg.id = str
    msg.succeed   = True
    pub.publish(msg)
    if ("right_arm" in str):
        last_action_right_arm = str 
    if ("left_arm" in str):
        last_action_left_arm = str 
    
    
def callback_collision_right(data):
    msg.id = last_action_right_arm
    collision_monitor.predicate = "collision_detected"
    collision_monitor.arguments = ["right_arm", "obstacle"]
    msg.monitors  = [collision_monitor]
    msg.succeed   = False
    pub.publish(msg)    

def callback_collision_left(data):
    msg.id = last_action_left_arm
    admittance_monitor.predicate = "collision_detected"
    admittance_monitor.arguments = ["left_arm", "obstacle"]
    msg.monitors  = [admittance_monitor]
    msg.succeed   = False
    pub.publish(msg)    

def callback_admittance_right(data):
    msg.id = last_action_right_arm
    collision_monitor.predicate = "admittance_detected"
    collision_monitor.arguments = ["right_arm"]
    msg.monitors  = [collision_monitor]
    msg.succeed   = False
    pub.publish(msg)    

def callback_admittance_left(data):
    msg.id = last_action_right_arm
    admittance_monitor.predicate = "admittance_detected"
    admittance_monitor.arguments = ["left_arm"]
    msg.monitors  = [admittance_monitor]
    msg.succeed   = False
    pub.publish(msg)    

def talker():
        
    pub = rospy.Publisher('action', Action, queue_size=10)
    rospy.Subscriber("eventPort", String, callback_action)    
    rospy.Subscriber("e_collision_right", String, callback_collision_right)    
    rospy.Subscriber("e_collision_left", String, callback_collision_left)    
    rospy.Subscriber("e_admittance_right", String, callback_admittance_right)    
    rospy.Subscriber("e_admittance_left", String, callback_admittance_left)    
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
