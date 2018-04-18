#!/usr/bin/env python
import rospy
from rospkg.rospack import RosPack
import os
import json
from os.path import join
from pobax_playground.srv import ExecuteVocalTrajectory, ExecuteVocalTrajectoryResponse
from pobax_playground.voice.arm_diva_env import PlayEnvironment


class VoiceNode(object):
    def __init__(self):
        self.rospack = RosPack()

        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'voice.json')) as f:
            self.params = json.load(f)

        # Serving these services
        self.service_name_execute = "/pobax_playground/voice/execute"

        # Init DIVA vocal tract simulator
        self.env = PlayEnvironment(self.params["tau"],
                                   self.params["pa"],
                                   self.params["pc"],
                                   gui=self.params["gui"],
                                   audio=self.params["audio"])

    def run(self):
        rospy.Service(self.service_name_execute, ExecuteVocalTrajectory, self.cb_execute)

    def cb_execute(self,request):
        rospy.loginfo('Voice node producing sound...')
        m = self.env.motor_babbling(audio=True)
        s = self.env.update(m)
        return ExecuteVocalTrajectoryResponse(sound_trajectory='hello, this is me again')

rospy.init_node('voice')
VoiceNode().run()
rospy.loginfo('%s node up and running' % rospy.get_name())
rospy.spin()