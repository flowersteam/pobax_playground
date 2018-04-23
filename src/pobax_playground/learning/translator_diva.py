from pobax_playground.tools.dmp.mydmp import MyDMP
from explauto.utils import bounds_min_max
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from pobax_playground.msg import  VocalTrajectory
from rospkg import RosPack
from os.path import join
import json
import numpy as np
import rospy


class EnvironmentTranslatorDiva(object):
    """
    This class gives sense to all the numerical parameters used by the learning and handles the transformation:
    Huge list of floats <=> meaningful class instances

    Therefore it also stores the joint names/order
    """
    def __init__(self):
        '''
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
        '''
        self.sound_baxter_size = 10
        self.sound_torso_size = 10
        self.sound_space_size = self.sound_baxter_size + self.sound_torso_size #Quick and Dirty, TODO JSON PARAMETERS

        self.arm_n_dims = 40 #Quick and Dirty, TODO JSON PARAMETERS
        self.diva_n_dims = 28 #Quick and Dirty, TODO JSON PARAMETERS
        self.sound_space_size = 20 #Quick and Dirty, TODO JSON PARAMETERS
        self.motor_space_size = 63 #Quick and Dirty, TODO JSON PARAMETERS
        # DMP PARAMETERS
        self.diva_use_initial = True
        self.diva_use_goal = True
        self.n_dmps = 7
        self.n_bfs = 2
        self.timesteps = 50

        self.max_params = []
        
        if self.diva_use_initial:
            self.max_params = self.max_params + [1.] * self.n_dmps
            self.max_params = self.max_params + [300.] * self.n_bfs * self.n_dmps
        if self.diva_use_goal:
            self.max_params = self.max_params + [1.] * self.n_dmps
        self.max_params = np.array(self.max_params)
        
        self.dmp = MyDMP(n_dmps=self.n_dmps, n_bfs=self.n_bfs, timesteps=self.timesteps, use_init=self.diva_use_initial, max_params=self.max_params)
        
        '''
        self.f0 = 1.
        self.pressure = 1.
        self.voicing = 1.
        self.art = np.array([0.]*10 + [self.f0, self.pressure, self.voicing])   # 13 articulators is a constant from diva_synth.m in the diva source code
        self.default_m = np.zeros(self.n_dmps * self.n_bfs + self.n_dmps * self.diva_use_initial + self.n_dmps * self.diva_use_goal)
        self.default_m_traj = self.compute_motor_command(self.default_m)
        self.default_sound = self.synth.execute(self.art.reshape(-1,1))[0]
        self.default_formants = None
        self.default_formants = self.compute_sensori_effect(self.default_m_traj)
        '''


    def trajectory_to_w(self, m_traj):
        assert m_traj.shape == (self.timesteps, self.n_dmps)
        normalized_traj = ((m_traj - self.bounds_motors_min) / (self.bounds_motors_max - self.bounds_motors_min)) * 2 + np.array([-1.]*self.n_dmps)
        return self.motor_dmp.imitate(normalized_traj) / self.max_params

    def w_to_trajectory(self, w):
        w = w[self.arm_n_dims:]
        #normalized_traj = bounds_min_max(self.motor_dmp.trajectory(np.array(w) * self.max_params), self.n_dmps * [-1.], self.n_dmps * [1.])
        return bounds_min_max(self.dmp.trajectory(np.array(w) * self.max_params), self.n_dmps * [-1.], self.n_dmps * [1.])
        #return  ((normalized_traj - np.array([-1.]*self.n_dmps))/2.) * (self.bounds_motors_max - self.bounds_motors_min) + self.bounds_motors_min

    def sensory_trajectory_msg_to_list(self, state):
        s_torso = state.s_response_torso_sound.data
        s_baxter = state.s_response_baxter_sound.data

        if len(s_torso) == 0:
            s_torso = [0.]*self.sound_torso_size
        if len(s_baxter) == 0:
            s_baxter = [0.]*self.sound_baxter_size
        return bounds_min_max(np.array(s_torso + s_baxter), 
                              self.sound_space_size*[-1.],
                              self.sound_space_size*[1.])

    def matrix_to_trajectory_msg(self, matrix_traj):
        assert matrix_traj.shape == (self.timesteps, self.n_dmps)
        return VocalTrajectory(data=matrix_traj.flatten())

    def trajectory_msg_to_matrix(self, trajectory):
        matrix = np.array([point.positions for point in trajectory.points])
        assert matrix.shape == (self.timesteps, self.n_dmps)
        return matrix
