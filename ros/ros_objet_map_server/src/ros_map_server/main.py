#!/usr/bin/env python3

import rospy

from ros_map_server_interfaces.msg import Example
from map_server import example_function

class MapServer():
    def __init__(self):
        rospy.init_node("ros_map_server", anonymous=True)
        rospy.loginfo("Starting")
        
        # Publisher
        #self.pub = rospy.Publisher(TOPIC, MSG, queue_size=10)

        # Subscriber
        #rospy.Subscriber(TOPIC, MSG, CALLBACK)

        #Execute main loop
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.loop()
            rate.sleep()

    def loop(self):
        '''
        main loop
        '''
        rospy.loginfo(Example())
        example_function()



if __name__ == '__main__':
    MapServer()

