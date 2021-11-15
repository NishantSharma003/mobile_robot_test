#!/usr/bin/env python
import rospy
from rospy_tutorials.msg import Floats
from mobile_robot_test.srv import custom_service_message, custom_service_messageResponse, custom_service_messageRequest

from mobile_robot_test.msg import custom_odom

#from geometry_msgs.msg import Pose

pos=0
vel=0

pos2=0
vel2=0



def my_callback(msg):
    global pos, vel, pos2, vel2 

    pos=msg.l_pose
    //vel=msg.l_vel
    pos2=msg.r_pose
    //vel2=msg.r_vel

    
    #print "vel: ", vel, "vel2: ", vel2 

def my_server(req):
    global pos, pos2
    res = custom_service_messageResponse() 
    res.res=[pos, pos2]
    return res

rospy.init_node('subscriber_py') #initialzing the node with name "subscriber_py"

rospy.Subscriber("/joint_states_from_arduino", custom_odom, my_callback) 
                                                                    # increased the buffer size from 10 to 100  

rospy.Service('/read_joint_state', custom_service_message, my_server)

rospy.spin() 
