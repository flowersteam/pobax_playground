from pobax_playground.tools.dmp.mydmp import MyDMP
from explauto.utils import bounds_min_max
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rospkg import RosPack
from os.path import join
import json
import numpy as np
import rospy


class EnvironmentTranslator(object):
    """
    This class gives sense to all the numerical parameters used by the learning and handles the transformation:
    Huge list of floats <=> meaningful class instances

    Therefore it also stores the joint names/order
    """
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'bounds.json')) as f:
            self.bounds = json.load(f)
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'learning.json')) as f:
            self.learning_params = json.load(f)
        self.sensory_state_size = self.learning_params['sensory_state_size']

        """TODO ADD ON SENSORY BOUNDS"""
        self.goal_spaces = ['hand','culbuto_1']
        self.bounds_motors_min = np.array([float(bound[0]) for bound in self.bounds['motors']['positions']])
        self.bounds_motors_max = np.array([float(bound[1]) for bound in self.bounds['motors']['positions']])
        self.bounds_sensory_min =  np.array([float(bounds[0]) for bounds in self.bounds['sensory']['culbuto_1']]
                                           +[d for space in self.goal_spaces for d in [float(bound[0])for bound in self.bounds['sensory'][space]]*10])
        self.bounds_sensory_max =  np.array([float(bounds[1]) for bounds in self.bounds['sensory']['culbuto_1']]
                                           +[d for space in self.goal_spaces for d in [float(bound[1])for bound in self.bounds['sensory'][space]]*10])
        self.bounds_sensory_diff = self.bounds_sensory_max - self.bounds_sensory_min

        # DMP PARAMETERS
        self.n_dmps = 5
        self.n_bfs = 7
        self.timesteps = 30
        self.max_params = np.array([300.] * self.n_bfs * self.n_dmps + [1.] * self.n_dmps)
        self.motor_dmp = MyDMP(n_dmps=self.n_dmps, n_bfs=self.n_bfs, timesteps=self.timesteps, max_params=self.max_params)
        self.context = {}
        self.config = dict(m_mins=[-1.]*40,
                           m_maxs=[1.]*40,
                           s_mins=[-1.]*self.sensory_state_size,
                           s_maxs=[1.]*self.sensory_state_size)

    def trajectory_to_w(self, m_traj):
        assert m_traj.shape == (self.timesteps, self.n_dmps)
        normalized_traj = ((m_traj - self.bounds_motors_min) / (self.bounds_motors_max - self.bounds_motors_min)) * 2 + np.array([-1.]*self.n_dmps)
        return self.motor_dmp.imitate(normalized_traj) / self.max_params

    def w_to_trajectory(self, w):
        normalized_traj = bounds_min_max(self.motor_dmp.trajectory(np.array(w) * self.max_params), self.n_dmps * [-1.], self.n_dmps * [1.])
        return  ((normalized_traj - np.array([-1.]*self.n_dmps))/2.) * (self.bounds_motors_max - self.bounds_motors_min) + self.bounds_motors_min

    def get_context(self, state):
        pos = state.culbuto_1.pose.position
        return [pos.x,pos.y,pos.z]

    def sensory_trajectory_msg_to_list(self, state):
        def flatten(list2d):
            return [element2 for element1 in list2d for element2 in element1]

        state_dict = {}
        state_dict['hand'] = flatten([(point.hand.pose.position.x, point.hand.pose.position.y, point.hand.pose.position.z) for point in state.points])
        state_dict['culbuto_1'] = flatten([(point.culbuto_1.pose.position.x, point.culbuto_1.pose.position.y, point.culbuto_1.pose.position.z) for point in state.points])

        self.context = {'culbuto_1': state_dict['culbuto_1'][:3]}
        rospy.loginfo("Context {}".format(self.context))

        assert len(state_dict['hand']) == 30, len(state_dict['hand'])
        assert len(state_dict['culbuto_1']) == 30, len(state_dict['culbuto_1'])

        # Concatenate all these values in a huge self.sensory_state_size-float list
        s_bounded = np.array(self.context['culbuto_1'] + [value for space in self.goal_spaces for value in state_dict[space]])
        s_normalized = ((s_bounded - self.bounds_sensory_min) / self.bounds_sensory_diff) * 2 + np.array([-1.]*self.sensory_state_size)
        s_normalized = bounds_min_max(s_normalized, self.sensory_state_size * [-1.], self.sensory_state_size * [1.])
        # print "context", s_bounded[:2], s_normalized[:2]
        # print "hand", s_bounded[2:32], s_normalized[2:32]
        # print "joystick_1", s_bounded[32:52], s_normalized[32:52]
        # print "joystick_2", s_bounded[52:72], s_normalized[52:72]
        # print "ergo", s_bounded[72:92], s_normalized[72:92]
        # print "ball", s_bounded[92:112], s_normalized[92:112]
        # print "light", s_bounded[112:122], s_normalized[112:122]
        # print "sound", s_bounded[122:self.sensory_state_size], s_normalized[122:self.sensory_state_size]

        return list(s_normalized)

    def matrix_to_trajectory_msg(self, matrix_traj):
        assert matrix_traj.shape == (self.timesteps, self.n_dmps)
        traj = JointTrajectory()
        traj.header.stamp = rospy.Time.now()
        traj.joint_names = ['l_shoulder_y', 'l_shoulder_x', 'l_arm_z', 'l_elbow_y', 'l_claw_x']
        traj.points = [JointTrajectoryPoint(positions=list(matrix_traj[point])) for point in range(len(matrix_traj))]
        return traj

    def trajectory_msg_to_matrix(self, trajectory):
        matrix = np.array([point.positions for point in trajectory.points])
        assert matrix.shape == (self.timesteps, self.n_dmps)
        return matrix
