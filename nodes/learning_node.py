#!/usr/bin/env python

import rospy
import os
import json
import datetime
from os.path import join
from rospkg.rospack import RosPack
from pobax_playground.srv import *
from pobax_playground.msg import Interests, Demonstration
from pobax_playground.learning import EnvironmentTranslator, Learning
from std_msgs.msg import String, Bool, UInt32, Float32
from threading import RLock
from copy import copy
from os.path import isfile


class LearningNode(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'learning.json')) as f:
            self.params = json.load(f)

        self.translator = EnvironmentTranslator()
        self.learning = Learning(self.translator.config,
                                 sensory_state_size=self.params["sensory_state_size"],
                                 n_motor_babbling=self.params["n_motor_babbling"], 
                                 explo_noise=self.params["explo_noise"], 
                                 choice_eps=self.params["choice_eps"], 
                                 enable_hand=self.params["enable_hand"],
                                 normalize_interests=self.params["normalize_interests"])

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
        self.pub_interests = rospy.Publisher('/pobax_playground/learning/interests', Interests, queue_size=1, latch=True)
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
            self.publish()
            rate.sleep()

    def publish(self):
        interests_array = self.learning.get_normalized_interests_evolution()
        interests = Interests()
        interests.names = self.learning.get_space_names()
        interests.num_iterations = UInt32(len(interests_array))
        interests.interests = [Float32(val) for val in interests_array.flatten()]

        self.pub_interests.publish(interests)


    ################################# Service callbacks
    def cb_perceive(self, request):
        #rospy.logwarn("Aborting perception, TODO modify learning core for new sensory space")
        #return PerceiveResponse()
        s = self.translator.sensory_trajectory_msg_to_list(request.demo.sensorial_demonstration)
        rospy.loginfo("Learning node is perceiving sensory trajectory for a demo")
        success = self.learning.perceive(s)
        if not success:
            rospy.logerr("Learner could not perceive this trajectory")
        return PerceiveResponse()

    def cb_produce(self, request):
        rospy.loginfo("Learning node is requesting the current state")
        state = self.get_state(GetSensorialStateRequest()).state

        rospy.loginfo("Learning node is producing...")
        w = self.learning.produce(self.translator.get_context(state))

        trajectory_matrix = self.translator.w_to_trajectory(w)
        trajectory_msg = self.translator.matrix_to_trajectory_msg(trajectory_matrix)

        self.ready_for_interaction = True

        response = ProduceResponse(torso_trajectory=trajectory_msg)
        return response

    def cb_save(self, request):
        rospy.loginfo("Learning node saving current state...")
        self.learning.save(self.experiment_file)
        rospy.loginfo("Saved file (periodic save) into {}".format(self.experiment_file))
        return SaveResponse()


if __name__ == '__main__':
    rospy.init_node('learning')
    LearningNode().run()