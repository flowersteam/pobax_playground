#!/usr/bin/env python
import rospy
from rospkg.rospack import RosPack
import os
import json
from os.path import join
from pobax_playground.srv import *
from pobax_playground.msg import SoundTrajectory
from pobax_playground.voice.voice import Voice


class VoiceNode(object):
    def __init__(self):
        self.rospack = RosPack()

        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'voice.json')) as f:
            self.params = json.load(f)

        # Serving these services
        self.service_name_execute_analyse = "/pobax_playground/voice/execute_analyse"
        self.service_name_baxter_analyse = "/pobax_playground/voice/baxter_analyse"

        # Init DIVA vocal tract simulator
        self.voice = Voice(self.params["tau"],
                         self.params["pa"],
                         self.params["pc"],
                         gui=self.params["gui"],
                         audio=self.params["audio"])

    def run(self):
        rospy.Service(self.service_name_execute_analyse, ExecuteAnalyseVocalTrajectory, self.cb_execute_analyse)
        rospy.Service(self.service_name_baxter_analyse, BaxterAnalyseVocalTrajectory, self.cb_baxter_analyse)

    def cb_baxter_analyse(self,request):
        rospy.loginfo('Voice node analysing baxter sound response to torso movement')
        baxter_sound_traj = self.voice.baxter_analyse(request.is_culbuto_touched)
        return BaxterAnalyseVocalTrajectoryResponse(baxter_sound_trajectory=SoundTrajectory(data=baxter_sound_traj))

    def cb_execute_analyse(self,request):
        rospy.loginfo('Voice node producing sound...')
        torso_sound_traj, baxter_sound_traj, is_culb_name, produced_name = self.voice.execute_analyse(request.vocal_trajectory.data)
        return ExecuteAnalyseVocalTrajectoryResponse(torso_sound_trajectory=SoundTrajectory(data=torso_sound_traj),
                                                     baxter_sound_trajectory=SoundTrajectory(data=baxter_sound_traj),
                                                     is_culbuto_name=is_culb_name,
                                                     produced_name=produced_name)

rospy.init_node('voice')
VoiceNode().run()
rospy.loginfo('%s node up and running' % rospy.get_name())
rospy.spin()