#!/usr/bin/env python
# The line above tells Linux that this file is a Python script, and that the OS
# should use the Python interpreter in /usr/bin/env to run it. Don't forget to
# use "chmod +x [filename]" to make this script executable.

# Import the dependencies as described in example_pub.py
import rospy
import tf2_ros
from std_msgs.msg import String
import sys

# Define the callback method which is called whenever this node receives a 
# message on its subscribed topic. The received message is passed as the first
# argument to callback(). 


# Define the method which contains the node's main functionality
def listener():
    
    tfBuffer = tf2_ros.Buffer()

    tfListener = tf2_ros.TransformListener(tfBuffer)

    source = sys.argv[1]

    target = sys.argv[2]

    print('SOURCE:', source)
    print('TARGET:', target)
    
    while not rospy.is_shutdown():
        try:

            trans = tfBuffer.lookup_transform(target,source,rospy.Time())
            print("TRANS:",trans)
            print("At Time:", rospy.get_time())
        except (tf2_ros.LookupException,tf2_ros.ConnectivityException,tf2_ros.ExtrapolationException) :
            print("EN ERROR HAS OCCURRED!!")
            

    
    # Create a new instance of the rospy.Subscriber object which we can use to
    # receive messages of type std_msgs/String from the topic /chatter_talk.
    # Whenever a new message is received, the method callback() will be called
    # with the received message as its first argument.
    
    # Wait for messages to arrive on the subscribed topics, and exit the node
    # when it is killed with Ctrl+C
    

# Python's syntax for a main() method
if __name__ == '__main__':

    # Run this program as a new node in the ROS computation graph called
    # /listener_<id>, where <id> is a randomly generated numeric string. This
    # randomly generated name means we can start multiple copies of this node
    # without having multiple nodes with the same name, which ROS doesn't allow.
    rospy.init_node('listener', anonymous=True)

    listener()
