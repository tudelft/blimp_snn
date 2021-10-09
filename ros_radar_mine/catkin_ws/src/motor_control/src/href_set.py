#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun May  9 16:20:25 2021

@author: marina
"""

import rospy
from std_msgs.msg import Float32

# GLOBAL VARS
HSET = [3.0, 3.0, 2.0, 1.0, 2.5, 1.5] # Go to each of these heights
T = 40                                # Publish evert T seconds

# Shutdown function print
def myhook():
  print("href_set node shutdown time!")

# Main
if __name__ == '__main__':
    
    rospy.init_node("href_set")                             # Node initialization
    rate = rospy.Rate(1/T)                                  # Pub rate
    pub = rospy.Publisher("/h_ref", Float32, queue_size=1)  # Publisher definition
    
    href_list = HSET
    lim = len(href_list)
    counter = 0
    
    # Publishing loop
    while not rospy.is_shutdown():
        
        if counter < lim:
            
            msg = Float32()
            msg.data = href_list[counter]
            pub.publish(msg)
            counter += 1
            rate.sleep()
        
        else:
            #rospy.on_shutdown(myhook)
            print("SHUTDOWN ROS")