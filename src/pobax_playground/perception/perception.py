import rospy
import json
import numpy as np
from os.path import join
from os import system
from rospkg.rospack import RosPack
from pobax_playground.srv import *
from pobax_playground.msg import SensorialState
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
        #self.service_name_baxter_start_record = "/pobax_playground/perception/baxter_start_record"
        #self.service_name_baxter_stop_record = "/pobax_playground/perception/baxter_stop_record"
        # Using these services
        self.topics = TopicAggregator()  # All topics are read and stored in that object

    def run(self):
        rospy.Service(self.service_name_get, GetSensorialState, self.cb_get)
        rospy.Service(self.service_name_record, Record, self.cb_record)
        #rospy.Service(self.service_name_baxter_start_record, BaxterStartRecord, self.cb_baxter_start_record)
        #rospy.Service(self.service_name_baxter_stop_record, BaxterStopRecord, self.cb_baxter_stop_record)

        rospy.loginfo("Done, perception is up!")

    def get(self):
        #TODO ADD VOICE
        state = SensorialState(hand=self.topics.torso_l_eef,
                               culbuto_1=self.topics.culbuto_1)
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
            response.sensorial_trajectory.points.append(self.get())
            self.rate.sleep()

        rospy.loginfo("Recorded!")
        return response
    '''
    def cb_baxter_start_record(self, request):
        return BaxterStartResponse

    def cb_baxter_stop_record(self, request):
        return BaxterStopResponse
    '''