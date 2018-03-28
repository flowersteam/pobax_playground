#!/usr/bin/env python
import rospy
import os
import json
import datetime
from os.path import join
from rospkg.rospack import RosPack
from pobax_playground.srv import *

class LearningNode(object):
    def __init__(self):

        # Serving these services
        self.service_name_perceive = "/pobax_playground/learning/perceive"
        self.service_name_produce = "/pobax_playground/learning/produce"

    def run(self):
        rospy.Service(self.service_name_produce, Produce, self.cb_produce)
        rospy.Service(self.service_name_perceive, Perceive, self.cb_perceive)
        rospy.loginfo("Learning is up!")
        rospy.spin()

    ################################# Service callbacks

    def cb_perceive(self, request):
        return PerceiveResponse()

    def cb_produce(self, request):
        return ProduceResponse("Produce skeleton,TODO implement trajectories, see translator from NIPS2016")


if __name__ == '__main__':
    rospy.init_node('learning')
    LearningNode().run()