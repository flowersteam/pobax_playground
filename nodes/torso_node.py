#!/usr/bin/env python
import rospy
from pobax_playground.torso import Torso

rospy.init_node('torso')
Torso().run()
