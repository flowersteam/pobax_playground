from std_msgs.msg import UInt8, Float32, Bool
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState, Joy
import rospy
import tf
from tf.transformations import euler_from_quaternion



class TopicAggregator(object):
    def __init__(self):
        self.topics = {
            "torso_l_j": {"topic": "/pobax_playground/torso/left_arm/joints", "sub": None, "data": JointState(), "type": JointState},
            "torso_l_eef": {"topic": "/pobax_playground/torso/left_arm/end_effector_pose", "sub": None, "data": PoseStamped(), "type": PoseStamped},
            "torso_r_eef": {"topic": "/pobax_playground/torso/right_arm/end_effector_pose", "sub": None, "data": PoseStamped(), "type": PoseStamped}     
        }

        self.topics["torso_l_j"]["sub"] = rospy.Subscriber(self.topics["torso_l_j"]["topic"], self.topics["torso_l_j"]["type"], self.cb_torso_l_j)
        self.topics["torso_l_eef"]["sub"] = rospy.Subscriber(self.topics["torso_l_eef"]["topic"], self.topics["torso_l_eef"]["type"], self.cb_torso_l_eef)
        self.topics["torso_r_eef"]["sub"] = rospy.Subscriber(self.topics["torso_r_eef"]["topic"], self.topics["torso_r_eef"]["type"], self.cb_torso_r_eef)

        # Init tf for optitracked objects
        self.listener = tf.TransformListener()

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
        try:
            position, orientation = self.listener.lookupTransform('/optitrack_frame','/culbuto/1', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logwarn("ERROR TF TRANSFORM IN AGGREGATOR")
        pose = PoseStamped()
        #WARNING this is not the same time as when the transform was actually acquired
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'optitrack_frame'
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

        pose.pose.orientation.x = orientation[0]
        pose.pose.orientation.y = orientation[1]
        pose.pose.orientation.z = orientation[2]
        pose.pose.orientation.w = orientation[3]

        return pose