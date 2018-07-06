import time
import numpy as np
import random
import matplotlib.pyplot as plt
import itertools
from os.path import join
from diva import Diva
from explauto.utils import bounds_min_max
from explauto.utils.utils import rand_bounds
import pickle
from rospkg.rospack import RosPack
import rospy  
import urllib2

class Voice(object):
    def __init__(self, tau, pa, pc, gui=False, audio=False):
        
        self.t = 0
        self.tau = tau
        self.pa = pa
        self.pc = pc
   
        # SOUND CONFIG
        self.v_o = list(np.log2([500, 900]))
        self.v_y = list(np.log2([300, 1700]))
        self.v_u = list(np.log2([300, 800]))
        self.v_e = list(np.log2([400, 2200]))
        self.v_i = list(np.log2([300, 2300]))

        
        # Retrieve caregiver sounds and trajectories from json
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'human_sounds.pickle')) as f:
            self.full_human_motor_traj, self.full_human_sounds_traj  = pickle.load(f)
        self.human_sounds = self.full_human_sounds_traj.keys()
        rospy.loginfo('Voice node using the word %s for culbuto name' % self.human_sounds[0])
        
        self.vowels = dict(o=self.v_o, y=self.v_y, u=self.v_u, e=self.v_e, i=self.v_i)
        
        self.human_sounds = ['eyu', 'oey', 'eou', 'oyi']
        #random.shuffle(self.human_sounds)
        print "human sounds", self.human_sounds

        
        def compress_sound_traj(sound):
            assert(len(sound) == 100)
            f1s = sound[:50]
            f3s = sound[50:]
            return np.append(f1s[np.array([0, 12, 24, 37, 49])],f3s[np.array([0, 12, 24, 37, 49])])
            
        
      
        self.human_sounds_traj = dict()
        self.human_sounds_traj_std = dict()
        self.best_vocal_errors = {}
        self.best_vocal_errors_evolution = []
        for hs in self.human_sounds:
            self.best_vocal_errors[hs] = 10.
            self.human_sounds_traj[hs] = compress_sound_traj(self.full_human_sounds_traj[hs])
            self.human_sounds_traj_std[hs] = [d - 8.5 for d in self.human_sounds_traj[hs][:5]] + [d - 10.25 for d in self.human_sounds_traj[hs][5:]]    
        
        '''
        def compute_s_sound(sound):
            s1 = self.vowels[sound[0]]
            s2 = [(self.vowels[sound[0]][0] + self.vowels[sound[1]][0]) / 2., (self.vowels[sound[0]][1] + self.vowels[sound[1]][1]) / 2.]
            s3 = self.vowels[sound[1]]
            s4 = [(self.vowels[sound[1]][0] + self.vowels[sound[2]][0]) / 2., (self.vowels[sound[1]][1] + self.vowels[sound[2]][1]) / 2.]
            s5 = self.vowels[sound[2]]
            rdm = 0.0 * (2.*np.random.random((1,10))[0] - 1.)
            return list(rdm + np.array([f[0] for f in [s1, s2, s3, s4, s5]] + [f[1] for f in [s1, s2, s3, s4, s5]]))
        
        
        self.human_sounds_traj = dict()
        self.human_sounds_traj_std = dict()
        self.best_vocal_errors = {}
        self.best_vocal_errors_evolution = []
        for hs in self.human_sounds:
            self.best_vocal_errors[hs] = 10.
            self.human_sounds_traj[hs] = compute_s_sound(hs)
            self.human_sounds_traj_std[hs] = [d - 8.5 for d in self.human_sounds_traj[hs][:5]] + [d - 10.25 for d in self.human_sounds_traj[hs][5:]]    
        '''

        
        self.diva_traj = None
        self.produced_sound = None
        self.sound_tol = 0.4

        self.audio = audio
        
        self.time_arm = 0.
        self.time_diva = 0. 
        self.time_arm_per_it = 0.
        self.time_diva_per_it = 0.

        self.n_dmps = 7 #Quick and Dirty, todo json
        self.timesteps = 50 #Quick and Dirty, todo json


        # DIVA CONFIG
        diva_cfg = dict(
                        m_mins = np.array([-1, -1, -1, -1, -1, -1, -1]),
                        m_maxs = np.array([1, 1, 1, 1, 1, 1, 1]),
                        s_mins = np.array([ 7.5,  9.25]),
                        s_maxs = np.array([ 9.5 ,  11.25]),
                        m_used = range(7),
                        s_used = range(1, 3),
                        rest_position_diva = list([0]*7),
                        audio = self.audio,
                        diva_use_initial = True,
                        diva_use_goal = True,
                        used_diva = list([True]*7),
                        n_dmps_diva = self.n_dmps,
                        n_bfs_diva = 2,
                        move_steps = self.timesteps,
                        )
        # Init Diva
        self.diva = Diva(**diva_cfg)

          
    def give_label(self, toy):
        
        if toy == "culbuto_1":
            #print "Caregiver says", self.human_sounds[0] 
            return self.human_sounds[0], self.human_sounds_traj_std[self.human_sounds[0]]
        elif toy == "distractor":
            sound_id = np.random.choice([1,2,3])
            #print "Caregiver says", self.human_sounds[sound_id]
            return self.human_sounds[sound_id], self.human_sounds_traj_std[self.human_sounds[sound_id]]            
        else:
            raise NotImplementedError
        
    def analysis_sound(self, diva_traj):
        #return self.human_sounds[2]
        #print self.human_sounds_traj
        for hs in self.human_sounds:          
            error = np.linalg.norm(np.array(self.human_sounds_traj[hs]) - np.array([f[0] for f in diva_traj[[0, 12, 24, 37, 49]]] + [f[1] for f in diva_traj[[0, 12, 24, 37, 49]]]))
            if error < self.best_vocal_errors[hs]:
                self.best_vocal_errors[hs] = error
            if error < self.sound_tol:
                print "***********Agent says", hs
                return hs
        return None
     
    def get_caregiver_answer(self, touched_toy):
        if self.pc:
            if touched_toy:
                return "contingent"
            else:
                return "distractor"
                
        pa, k, theta, l = self.pa, 5., 0.25, np.log(2.)/2.
        def distractor_answer(l): return np.random.exponential(1./l)
        def caregiver_answer(pa, k, theta):          
            if not touched_toy or np.random.random() < 1 - pa:
                return None
            else:                                  
                return np.random.gamma(k, theta) 
       
        c = caregiver_answer(pa, k, theta)
            
        d = distractor_answer(l)
        if c is not None and c < d and c < self.tau:
            #print "contingent answer"
            return "contingent"
        elif d < self.tau:
#             if touched_toy:
#                 print "distractor answer"
            return "distractor"
#         else:
#             if touched_toy:
#                 print "no answer"


    def baxter_analyse(self, is_culbuto_touched):
        # CAREGIVER VOCAL REACTION
        caregiver_answer = self.get_caregiver_answer(is_culbuto_touched)
        #print caregiver_answer
        sound_id = None
        if caregiver_answer == "contingent":
            if is_culbuto_touched:
                rospy.logwarn("CULBUTO WAS TOUCHED HIP HIP HIP HOURRRAY")
                sound_id, self.caregiver_sound = self.give_label("culbuto_1")

        elif caregiver_answer == "distractor":
            sound_id, self.caregiver_sound = self.give_label("distractor")
        else:
            # No sound
            self.caregiver_sound = None

        if not sound_id == None: # A word was choosen by caregiver
            rospy.logwarn('Voice Node: Baxter says %s' % sound_id)
            if self.audio: # Play the word
                url= "http://193.50.110.54:8080/api/v1/"+sound_id
                contents = urllib2.urlopen(url).read()
                #self.diva.compute_sensory_effect(self.full_human_motor_traj[sound_id],sound_power=30.)
        return self.caregiver_sound

    # Generate and play sound from diva trajectory
    # Returns a 10-D sound trajectory for baxter and torso
    # And a boolean set to True if the sound was culbuto's name      
    def execute_analyse(self, diva_traj):
        t = time.time()

        #Reshape trajectory list into matrix
        diva_traj = np.reshape(diva_traj, (self.timesteps,self.n_dmps))

        is_culbuto_name = False

        self.raw_produced_sound = self.diva.compute_sensory_effect(diva_traj)      
        self.produced_sound = self.analysis_sound(self.raw_produced_sound)
        if self.produced_sound is not None:
            # parent gives object if label is produced and object out of reach
            if self.produced_sound == self.human_sounds[0]:
                is_culbuto_name = True
                rospy.logwarn("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!CULBUTO NAME WAS PRONOUNCED")

        self.self_sound = [f for formants in self.raw_produced_sound[[0, 12, 24, 37, 49]] for f in formants]
        self.self_sound = self.self_sound[0::2] + self.self_sound[1::2]
        # MAP TO STD INTERVAL (only extracts first 2 formants)
        self.downsampled_self_sound = [d - 8.5 for d in self.self_sound[:5]] + [d - 10.25 for d in self.self_sound[5:]]
        #self.downsampled_caregiver_sound = [d - 8.5 for d in self.caregiver_sound[:5]] + [d - 10.25 for d in self.caregiver_sound[5:]]

        #return bounds_min_max(s, self.conf.s_mins, self.conf.s_maxs)
        flat_raw_produced_sound =  [f for formants in self.raw_produced_sound for f in formants]
        return self.downsampled_self_sound, None, is_culbuto_name, self.produced_sound, flat_raw_produced_sound
        
        # MAP TO STD INTERVAL
        #self_sound = [d - 8.5 for d in self.self_sound[:5]] + [d - 10.25 for d in self.self_sound[5:]]
        #caregiver_sound = [d - 8.5 for d in self.caregiver_sound[:5]] + [d - 10.25 for d in self.caregiver_sound[5:]]

        #return bounds_min_max(s, self.conf.s_mins, self.conf.s_maxs)