from std_msgs.msg import UInt8, Float32, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy
import rospy



class TopicAggregator(object):
    def __init__(self):
        self.topics = {
            "torso_l_j": {"topic": "/pobax_playground/torso/left_arm/joints", "sub": None, "data": JointState(), "type": JointState},
            "torso_l_eef": {"topic": "/pobax_playground/torso/left_arm/end_effector_pose", "sub": None, "data": PoseStamped(), "type": PoseStamped},
            "torso_r_eef": {"topic": "/pobax_playground/torso/right_arm/end_effector_pose", "sub": None, "data": PoseStamped(), "type": PoseStamped}
        }


        self.topics["torso_l_j"]["sub"] = rospy.Subscriber(self.topics["torso_l_j"]["topic"], self.topics["torso_l_j"]["type"], self.cb_torso_l_j)
        #self.topics["torso_l_eef"]["sub"] = rospy.Subscriber(self.topics["torso_l_eef"]["topic"], self.topics["torso_l_eef"]["type"], self.cb_torso_l_eef)
        #self.topics["torso_r_eef"]["sub"] = rospy.Subscriber(self.topics["torso_r_eef"]["topic"], self.topics["torso_r_eef"]["type"], self.cb_torso_r_eef)

    def cb_torso_l_j(self, msg):
        self.topics["torso_l_j"]["data"] = msg

    def cb_torso_l_eef(self, msg):
        self.topics["torso_l_eef"]["data"] = msg

    def cb_torso_r_eef(self, msg):
        self.topics["torso_r_eef"]["data"] = msg



    @property
    def torso_l_j(self):
        return self.topics["torso_l_j"]["data"]

    @property
    def torso_l_eef(self):
        return self.topics["torso_l_eef"]["data"]

    @property
    def torso_r_eef(self):
        return self.topics["torso_r_eef"]["data"]

    @property
    def culbuto_1(self):
        return self.topics["torso_r_eef"]["data"]