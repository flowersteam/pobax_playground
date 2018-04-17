#! /usr/bin/env python

import rospy
import actionlib
from pobax_playground.msg import *
from pobax_playground.perception import Perception
from pobax_playground.perception import TopicAggregator
from rospkg.rospack import RosPack
import json
import numpy as np
from os.path import join

# An action server used to perform a non-blocking recording of the sensorial state
# (here it is used to record culbuto's position while baxter is replacing it)
class RecordActionServer(object):
    # create messages that are used to publish feedback/result
    _feedback = RecordFeedback()
    _result = RecordResult()

    def __init__(self):
        self._action_name = '/pobax_playground/perception/record_server'
        self._as = actionlib.SimpleActionServer(self._action_name, RecordAction, execute_cb=self.execute_cb, auto_start = False)
        self._as.start()
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)
        self.rate = rospy.Rate(self.params['recording_rate'])

        self.topics = TopicAggregator()
     
    def get(self):
        #TODO ADD VOICE
        state = SensorialState(hand=self.topics.torso_l_eef,
                               culbuto_1=self.topics.culbuto_1)
        return state

    # Choose nb_points from sensorial_demo, evenly spaced 
    # (except for last point which is always the last one recorded)
    def choose_points(self, sensorial_traj, nb_points):
        sequence_size = len(sensorial_traj.points) 
        if sequence_size < nb_points:
            rospy.logerr('%s: Recording time was not long enough to gather enough data points: recorded: %s vs asked: %s' % (self._action_name, sequence_size, nb_points))
            return
        # Handy fonction for choosing m evenly spaced elements from a sequence of length n
        f = lambda m, n: [i*n//m + n//(2*m) for i in range(m)]
        choosen_idx = f(nb_points, sequence_size)
        # Change the last point in choosen sequence by the last recorder point
        choosen_idx[-1] = (sequence_size-1)
        result = SensorialTrajectory()
        for i in choosen_idx:
            result.points.append(sensorial_traj.points[i])
        return result

    def execute_cb(self, goal):
        rospy.loginfo('%s: Start non-blocking recording of culbuto' % (self._action_name))
        sensorial_demo = SensorialTrajectory()
        while not rospy.is_shutdown():
            if self._as.is_preempt_requested(): # When requested, we stop registering positions
                rospy.loginfo('%s: Preempted' % self._action_name)
                break
            sensorial_demo.points.append(self.get())
            self.rate.sleep()
          
        self._result.sensorial_trajectory = self.choose_points(sensorial_demo, goal.nb_points.data)
        rospy.loginfo('%s: Succeeded' % self._action_name)
        self._as.set_succeeded(self._result)

if __name__ == '__main__':
    rospy.init_node('perception')
    server = RecordActionServer()
    Perception().run()
    rospy.spin()