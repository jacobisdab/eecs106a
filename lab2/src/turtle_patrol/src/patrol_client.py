#!/usr/bin/env python
import numpy as np
import rospy
from turtle_patrol.srv import Patrol  # Import service type
import sys



turtle_name = sys.argv[1]
vel = sys.argv[2]
omega = sys.argv[3]
x_pos = sys.argv[4]
y_pos = sys.argv[5]
theta = sys.argv[6]
def patrol_client():


    

    # Initialize the client node
    rospy.init_node(f'{turtle_name}_patrol_client')
    # Wait until patrol service is ready
    rospy.wait_for_service(f'/{turtle_name}/patrol')
    try:
        # Acquire service proxy
        patrol_proxy = rospy.ServiceProxy(
            f'/{turtle_name}/patrol', Patrol)
        
        rospy.loginfo(f'Command {turtle_name} to patrol')
        # Call patrol service via the proxy
        patrol_proxy(vel, omega,x_pos,y_pos,theta)
    except rospy.ServiceException as e:
        rospy.loginfo(e)


if __name__ == '__main__':
    patrol_client()

