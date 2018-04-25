#!/usr/bin/env python

import rospy
import os
import json
import datetime
from os.path import join
from rospkg.rospack import RosPack
from pobax_playground.srv import *
from pobax_playground.msg import Interests
from pobax_playground.learning import EnvironmentTranslatorDiva, EnvironmentTranslatorArm, Learning
from std_msgs.msg import String, Bool, UInt32, Float32
from threading import RLock
from copy import copy
from os.path import isfile
import numpy as np


class LearningNode(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'learning.json')) as f:
            self.params = json.load(f)
        #with open(join(self.rospack.get_path('pobax_playground'), 'config', 'dmps.json')) as f:
        #    self.dmps_params = json.load(f)

        self.arm_translator = EnvironmentTranslatorArm()
        self.diva_translator = EnvironmentTranslatorDiva()
        self.config = dict(m_mins=[-1.]*68,
                           m_maxs=[1.]*68,
                           s_mins=[-1.]*self.params["sensory_state_size"],
                           s_maxs=[1.]*self.params["sensory_state_size"])
        self.learning = Learning(self.config,
                                 sensory_state_size=self.params["sensory_state_size"],
                                 model_babbling=self.params["model_babbling"],
                                 n_motor_babbling=self.params["n_motor_babbling"], 
                                 explo_noise=self.params["explo_noise"], 
                                 choice_eps=self.params["choice_eps"], 
                                 proba_imitate=self.params["proba_imitate"],
                                 tau=self.params["tau"])

        self.experiment_name = rospy.get_param("/pobax_playground/experiment_name", "experiment")
        rospy.loginfo("Learning node will write {}".format(self.experiment_name))

        # Saved experiment files
        self.dir = join(self.rospack.get_path('pobax_playground'), 'logs')
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)

        self.experiment_file = join(self.dir, self.experiment_name + '.pickle')

        starting_iteration = 0
        if isfile(self.experiment_file):
            starting_iteration = self.learning.restart_from_end_of_file(self.experiment_file)
        else: 
            self.learning.start()
        rospy.set_param("/pobax_playground/starting_iteration", starting_iteration)

        # Serving these services
        self.service_name_perceive = "/pobax_playground/learning/perceive"
        self.service_name_produce = "/pobax_playground/learning/produce"
        self.service_name_save = "/pobax_playground/learning/save"

        # Publishing these topics
        #self.pub_interests = rospy.Publisher('/pobax_playground/learning/interests', Interests, queue_size=1, latch=True)
        self.pub_focus = rospy.Publisher('/pobax_playground/learning/current_focus', String, queue_size=1, latch=True)

        
        # Using these services
        self.service_name_get_perception = "/pobax_playground/perception/get"
        for service in [self.service_name_get_perception]:
            rospy.loginfo("Learning  node is waiting service {}...".format(service))
            rospy.wait_for_service(service)
        self.get_state = rospy.ServiceProxy(self.service_name_get_perception, GetSensorialState)
        
    def run(self):
        rospy.Service(self.service_name_perceive, Perceive, self.cb_perceive)
        rospy.Service(self.service_name_produce, Produce, self.cb_produce)
        rospy.Service(self.service_name_save, Save, self.cb_save)
        rospy.loginfo("Learning is up!")

        rate = rospy.Rate(self.params['publish_rate'])
        while not rospy.is_shutdown():
            #self.publish()
            rate.sleep()
    '''
    def publish(self):
        interests_array = self.learning.get_normalized_interests_evolution()
        interests = Interests()
        interests.names = self.learning.get_space_names()
        interests.num_iterations = UInt32(len(interests_array))
        interests.interests = [Float32(val) for val in interests_array.flatten()]

        self.pub_interests.publish(interests)
    '''

    ################################# Service callbacks
    def cb_perceive(self, request):
        #rospy.logwarn("Aborting perception, TODO modify learning core for new sensory space")
        #return PerceiveResponse()
        s_physical = self.arm_translator.sensory_trajectory_msg_to_list(request)
        s_sound = self.diva_translator.sensory_trajectory_msg_to_list(request)
        rospy.loginfo("Learning node is perceiving sensory trajectory")
        #print "physical"
        #print s_physical
        #print "sound"
        #print s_sound
        success = self.learning.perceive(np.append(s_physical, s_sound))
        if not success:
            rospy.logerr("Learner could not perceive this trajectory")
        return PerceiveResponse()

    def cb_produce(self, request):
        rospy.loginfo("Learning node is requesting the current state")
        state = self.get_state(GetSensorialStateRequest()).state

        rospy.loginfo("Learning node is producing...")
        w, param_type = self.learning.produce(self.arm_translator.get_context(state))

        produce_msg = ProduceResponse(trajectory_type=param_type)
        if param_type == "arm":
            trajectory_matrix = self.arm_translator.w_to_trajectory(w)
            trajectory_msg = self.arm_translator.matrix_to_trajectory_msg(trajectory_matrix)
            produce_msg.torso_trajectory = trajectory_msg
        elif param_type == "diva":
            trajectory_matrix = self.diva_translator.w_to_trajectory(w)
            trajectory_msg = self.diva_translator.matrix_to_trajectory_msg(trajectory_matrix)
            produce_msg.vocal_trajectory = trajectory_msg
        else:
            rospy.logerr("Learning Node received an unknown param_type when calling produce()")

        self.ready_for_interaction = True
        return produce_msg

    def cb_save(self, request):
        rospy.loginfo("Learning node saving current state...")
        self.learning.save(self.experiment_file)
        rospy.loginfo("Saved file (periodic save) into {}".format(self.experiment_file))
        return SaveResponse()


if __name__ == '__main__':
    rospy.init_node('learning')
    LearningNode().run()