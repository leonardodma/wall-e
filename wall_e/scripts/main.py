#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
from robo import *

v = 0.35  # Velocidade linear
w = 0.2  # Velocidade angular

#robo = Robot(v, w, "blue", 12, "dog")
robo = Robot(v, w, "green", 23, "horse")
#robo = Robot(v, w, "orange", 11, "cow")

if __name__ == "__main__":
    rospy.init_node("wall_e")

    try:
        while not rospy.is_shutdown():
            robo.main()

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")