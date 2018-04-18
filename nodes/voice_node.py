#!/usr/bin/env python
import rospy
from rospkg.rospack import RosPack
import os
import json
from os.path import join

class VoiceNode(object):
    def __init__(self):
        self.rospack = RosPack()

        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'voice.json')) as f:
            self.params = json.load(f)

rospy.init_node('voice')
VoiceNode()
rospy.loginfo('%s node up and running' % rospy.get_name())
rospy.spin()