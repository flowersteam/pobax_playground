#!/usr/bin/env python
import rospy
from pobax_playground.perception import Perception

rospy.init_node('perception')
Perception().run()
rospy.spin()