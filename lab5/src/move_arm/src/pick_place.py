#!/usr/bin/env python
import rospy
from moveit_msgs.srv import GetPositionIK, GetPositionIKRequest, GetPositionIKResponse
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander
import numpy as np
from numpy import linalg
import sys
import intera_interface
from intera_interface import gripper as robot_gripper


def main():
    # Wait for the IK service to become available
    rospy.wait_for_service('compute_ik')
    rospy.init_node('service_query')


    # Set up the right gripper
    right_gripper = robot_gripper.Gripper('right_gripper')




    # Create the function used to call the service
    compute_ik = rospy.ServiceProxy('compute_ik', GetPositionIK)
    while not rospy.is_shutdown():
        input('Press [ Enter ]: ')
        
        # Construct the request
        request = GetPositionIKRequest()
        request.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request.ik_request.pose_stamped.pose.position.x = 0.707
        request.ik_request.pose_stamped.pose.position.y = -0.206
        request.ik_request.pose_stamped.pose.position.z = -0.091     
        request.ik_request.pose_stamped.pose.orientation.x = 0.0
        request.ik_request.pose_stamped.pose.orientation.y = 1.0
        request.ik_request.pose_stamped.pose.orientation.z = 0.0
        request.ik_request.pose_stamped.pose.orientation.w = 0.0



        #position of cube before pick and place


        request2 = GetPositionIKRequest()
        request2.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request2.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request2.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request2.ik_request.pose_stamped.pose.position.x = 0.641
        request2.ik_request.pose_stamped.pose.position.y = 0.105
        request2.ik_request.pose_stamped.pose.position.z = -0.099      
        request2.ik_request.pose_stamped.pose.orientation.x = 0.0
        request2.ik_request.pose_stamped.pose.orientation.y = 1.0
        request2.ik_request.pose_stamped.pose.orientation.z = 0.0
        request2.ik_request.pose_stamped.pose.orientation.w = 0.0


        request3 = GetPositionIKRequest()
        request3.ik_request.group_name = "right_arm"

        # If a Sawyer does not have a gripper, replace '_gripper_tip' with '_wrist' instead
        link = "right_gripper_tip"

        request3.ik_request.ik_link_name = link
        # request.ik_request.attempts = 20
        request3.ik_request.pose_stamped.header.frame_id = "base"
        
        # Set the desired orientation for the end effector HERE
        request3.ik_request.pose_stamped.pose.position.x = 0.691
        request3.ik_request.pose_stamped.pose.position.y = 0.158
        request3.ik_request.pose_stamped.pose.position.z = 0.423      
        request3.ik_request.pose_stamped.pose.orientation.x = 0.0
        request3.ik_request.pose_stamped.pose.orientation.y = 1.0
        request3.ik_request.pose_stamped.pose.orientation.z = 0.0
        request3.ik_request.pose_stamped.pose.orientation.w = 0.0


        


        
        try:
            # Send the request to the service
            response = compute_ik(request)
            
            # Print the response HERE
            print(response)
            group = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group.set_pose_target(request.ik_request.pose_stamped)

            # Plan IK
            plan = group.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':
                print('Calibrating...')
                right_gripper.calibrate()
                rospy.sleep(2.0)

                # Open the right gripper
                print('Opening...')
                right_gripper.open()
                rospy.sleep(1.0)


                group.execute(plan[1])

                rospy.sleep(1.0)

                # Calibrate the gripper (other commands won't work unless you do this first)
                print('Calibrating...')
                right_gripper.calibrate()
                rospy.sleep(2.0)

                # Close the right gripper
                print('Closing...')
                right_gripper.close()
                rospy.sleep(1.0)
            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


        try:
            # Send the request to the service
            response1 = compute_ik(request3)
            
            # Print the response HERE
            print(response1)
            group1 = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group1.set_pose_target(request3.ik_request.pose_stamped)

            # Plan IK
            plan = group1.plan()
            user_input = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input == 'y':

                group1.execute(plan[1])

            
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        


















        try:
            # Send the request to the service
            response2 = compute_ik(request2)
            
            # Print the response HERE
            print(response2)
            group2 = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group2.set_pose_target(request2.ik_request.pose_stamped)

            # Plan IK
            plan2 = group2.plan()
            user_input2 = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input2 == 'y':
                group2.execute(plan2[1])
                print('Calibrating...')
                right_gripper.calibrate()
                rospy.sleep(2.0)

                # Open the right gripper
                print('Opening...')
                right_gripper.open()
                rospy.sleep(1.0)
                print('Done!')


        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)
        
        try:
            # Send the request to the service
            response3 = compute_ik(request3)
            
            # Print the response HERE
            print(response3)
            group3 = MoveGroupCommander("right_arm")

            # Setting position and orientation target
            group3.set_pose_target(request3.ik_request.pose_stamped)

            # Plan IK
            plan3 = group3.plan()
            user_input3 = input("Enter 'y' if the trajectory looks safe on RVIZ")
            
            # Execute IK if safe
            if user_input3 == 'y':
                group3.execute(plan3[1])
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

# Python's syntax for a main() method
if __name__ == '__main__':
    main()
