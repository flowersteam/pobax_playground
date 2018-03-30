import rospy
import json
import numpy as np
from os.path import join
from os import system
from rospkg.rospack import RosPack
from pobax_playground.srv import *
from pobax_playground.msg import SensorialState, Demonstration
from .aggregator import TopicAggregator
from ..tools import joints


class Perception(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)
        self.rate = rospy.Rate(self.params['recording_rate'])

        # Serving these services
        self.service_name_get = "/pobax_playground/perception/get"
        self.service_name_record = "/pobax_playground/perception/record"
        # Using these services
        self.topics = TopicAggregator()  # All topics are read and stored in that object

    def run(self):
        rospy.Service(self.service_name_get, GetSensorialState, self.cb_get)
        rospy.Service(self.service_name_record, Record, self.cb_record)
        rospy.loginfo("Done, perception is up!")

    def get(self):
        state = SensorialState(culbuto_1=self.topics.culbuto_1)
                               #hand=self.topics.torso_l_eef)
        return state

    ################################# Service callbacks
    def cb_get(self, request):
        return GetSensorialStateResponse(state=self.get())

    def cb_record(self, request):
        response = RecordResponse()
        # TODO eventually keep trace of the last XX points to start recording prior to the start signal
        # TODO add VOICE DEMO
        rospy.loginfo("Recording {}...".format("an arm demo"))
        for point in range(request.nb_points.data):
            if rospy.is_shutdown():
                break
            if point % self.params["divider_nb_points_sensory"] == 0:
                response.demo.sensorial_demonstration.points.append(self.get())
            if not is_joystick_demo:
                response.demo.torso_demonstration.points.append(joints.state_to_jtp(self.topics.torso_l_j))
            self.rate.sleep()

        response.demo.type_demo = Demonstration.TYPE_DEMO_NORMAL
        rospy.loginfo("Recorded!")
        return response
