#!/usr/bin/env python
import rospy
from thr_infrastructure_msgs.msg import RunRobotActionActionResult
from pobax_playground.srv import BaxterResult, BaxterResultResponse
from rospkg import RosPack




class BaxterNode(object):
    def __init__(self):
        self.rospack = RosPack()

        # Using this topic
        self.topic_name_result = "/thr/robot_run_action/right/result"
        self._baxter_result = None

        # Serving these services
        self.service_name_get_result = "/pobax_playground/baxter/get_result"

    def cb_baxter_result(self, msg):
        self._baxter_result = msg

    def cb_baxter_get_result(self,request):
        return BaxterResultResponse(self.baxter_result)

    @property
    def baxter_result(self):
        return self._baxter_result

    def run(self):
        rospy.Subscriber(self.topic_name_result,RunRobotActionActionResult, self.cb_baxter_result)
        rospy.Service(self.service_name_get_result, BaxterResult, self.cb_baxter_get_result)
        rospy.spin()

if __name__ == '__main__':
    rospy.init_node('baxter')
    BaxterNode().run()