import numpy as np
import time
from explauto.utils import rand_bounds, bounds_min_max, softmax_choice, prop_choice
from explauto.utils.config import make_configuration
from learning_module import LearningModule


class Supervisor(object):
    def __init__(self, config, sensory_state_size, n_motor_babbling=0, explo_noise=0.1, choice_eps=0.2, enable_hand=True, normalize_interests=True):
        
        self.config = config
        self.sensory_state_size = sensory_state_size
        self.n_motor_babbling = n_motor_babbling
        self.explo_noise = explo_noise
        self.choice_eps = choice_eps,
        self.enable_hand = enable_hand
        self.normalize_interests = normalize_interests
        
        self.conf = make_configuration(**config)
        
        self.t = 0
        self.modules = {}
        self.chosen_modules = []
        self.progresses_evolution = {}
        self.interests_evolution = {}
        
        self.have_to_replay_arm_demo = None
            
        self.mid_control = ''
        
        # Define motor and sensory spaces:
        m_ndims = self.conf.m_ndims # number of motor parameters
        
        self.m_space = range(m_ndims)
        self.c_dims = range(m_ndims, m_ndims+3)
        self.s_hand = range(m_ndims+3, m_ndims+33)
        self.s_culbuto_1 = range(m_ndims+33, m_ndims+63)
      
        self.s_spaces = dict(s_hand=self.s_hand, 
                             s_culbuto_1=self.s_culbuto_1)
        
        # Create the 6 learning modules:
        self.modules['mod1'] = LearningModule("mod1",
                                              self.m_space, 
                                              self.s_hand, 
                                              self.conf, 
                                              explo_noise=self.explo_noise, 
                                              normalize_interests=self.normalize_interests)
        self.modules['mod2'] = LearningModule("mod4",
                                              self.m_space,
                                              self.c_dims + self.s_culbuto_1,
                                              self.conf,
                                              context_mode=dict(mode='mcs', 
                                              context_n_dims=3, context_sensory_bounds=[[-2.,2.],[-2.,2.],[-2.,2.]]),
                                              explo_noise=self.explo_noise,
                                              normalize_interests=self.normalize_interests)
        self.space2mid = dict(s_hand="mod1",
                              s_culbuto_1="mod2") 
         
        self.mid2space = dict(mod1="s_hand", 
                              mod2="s_culbuto_1")
        
        for mid in self.modules.keys():
            self.progresses_evolution[mid] = []
            self.interests_evolution[mid] = []
    
    def mid_to_space(self, mid): return self.mid2space[mid]
    def space_to_mid(self, space): return self.space2mid[space]
    def get_space_names(self): return ["s_hand", "s_culbuto_1"]
    def get_last_focus(self): return self.mid_to_space(self.mid_control) if self.mid_control else ""
    
    def save(self):
        sm_data = {}
        im_data = {}
        for mid in self.modules.keys():
            sm_data[mid] = self.modules[mid].sensorimotor_model.save()
            im_data[mid] = self.modules[mid].interest_model.save()            
        return {"sm_data":sm_data,
                "im_data":im_data,
                "chosen_modules":self.chosen_modules,
                "progresses_evolution":self.progresses_evolution,
                "interests_evolution":self.interests_evolution,
                "normalized_interests_evolution":self.get_normalized_interests_evolution(),
                "normalize_interests":self.normalize_interests}
 
    def forward(self, data, iteration):
        if iteration > len(data["chosen_modules"]):
            print "\nWARNING: asked to restart from iteration", iteration, "but only", len(data["chosen_modules"]), "are available. Restarting from iteration", len(data["chosen_modules"]), "..."
            iteration = len(data["chosen_modules"])
        if iteration < 0:
            iteration = len(data["chosen_modules"])
        self.chosen_modules = data["chosen_modules"][:iteration]
        self.progresses_evolution = data["progresses_evolution"]
        self.interests_evolution = data["interests_evolution"]
        for mid in self.modules.keys():
            self.progresses_evolution[mid] = self.progresses_evolution[mid][:iteration]
            self.interests_evolution[mid] = self.interests_evolution[mid][:iteration]
        self.t = iteration
        if data.has_key("normalize_interests"):
            self.normalize_interests = data["normalize_interests"]
        if iteration > 0:
            for mid in self.modules.keys():
                if mid == "mod1":
                    self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration-self.chosen_modules.count("j_demo"))
                else:
                    self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration)
                    
                self.modules[mid].interest_model.forward(data["im_data"][mid], self.chosen_modules.count(mid), self.progresses_evolution[mid][-1], self.interests_evolution[mid][-1])

    def choose_babbling_module(self, mode='prop'):
        interests = {}
        for mid in self.modules.keys():
            if not ((not self.enable_hand) and mid=="mod1"):
                interests[mid] = self.modules[mid].interest()
            else:
                interests[mid] = 0.
        
        if mode == 'random':
            mid = np.random.choice(self.interests.keys())
        elif mode == 'greedy':
            eps = 0.2
            if np.random.random() < eps:
                mid = np.random.choice(self.interests.keys())
            else:
                mid = max(interests, key=interests.get)
        elif mode == 'softmax':
            temperature = 0.1
            w = interests.values()
            mid = self.modules.keys()[softmax_choice(w, temperature)]
        
        elif mode == 'prop':
            w = interests.values()
            mid = self.modules.keys()[prop_choice(w, eps=self.choice_eps)]
        
        self.chosen_modules.append(mid)
        return mid
        
    def fast_forward(self, log, forward_im=False):
        #ms_list = []
        for m,s in zip(log.logs['motor'], log.logs['sensori']):
            ms = np.append(m,s)
            self.update_sensorimotor_models(ms)
            #ms_list += [ms]
        for mid, mod in self.modules.iteritems():
            mod.fast_forward_models(log, ms_list=None, from_log_mod=mid, forward_im=forward_im)        
        
    def eval_mode(self): 
        self.sm_modes = {}
        for mod in self.modules.values():
            self.sm_modes[mod.mid] = mod.sensorimotor_model.mode
            mod.sensorimotor_model.mode = 'exploit'
                
    def learning_mode(self): 
        for mod in self.modules.values():
            mod.sensorimotor_model.mode = self.sm_modes[mod.mid]
                
    def check_bounds_dmp(self, m_ag):return bounds_min_max(m_ag, self.conf.m_mins, self.conf.m_maxs)
    def motor_primitive(self, m): return m
    def sensory_primitive(self, s): return s
    def get_m(self, ms): return ms[self.conf.m_dims]
    def get_s(self, ms): return ms[self.conf.s_dims]
    
    def motor_babbling(self):
        self.m = self.modules["mod1"].motor_babbling()
        return self.m
    
    def set_ms(self, m, s): return np.array(list(m) + list(s))
            
    def update_sensorimotor_models(self, ms):
        for mid in self.modules.keys():
            self.modules[mid].update_sm(self.modules[mid].get_m(ms), self.modules[mid].get_s(ms))
        
    def increase_interest(self, mid):
        self.modules[mid].interest_model.current_progress = self.modules[mid].interest_model.current_progress * 1.1
        self.modules[mid].interest_model.current_interest = abs(self.modules[mid].interest_model.current_progress)
            
    def produce(self, context):
        if self.t < self.n_motor_babbling:
            self.mid_control = None
            self.chosen_modules.append("motor_babbling")
            return self.motor_babbling()
        else:
            mid = self.choose_babbling_module()
            self.mid_control = mid
            
            if self.modules[mid].context_mode is None:
                self.m = self.modules[mid].produce()
            else:
                self.m = self.modules[mid].produce(context=np.array(context)[range(self.modules[mid].context_mode["context_n_dims"])])
            return self.m
    
    def inverse(self, mid, s, context):
        if self.modules[mid].context_mode is not None:
            s = np.array(list(context[:self.modules[mid].context_mode["context_n_dims"]]) + list(s))
        else:
            s = np.array(s)
        self.mid_control = None
        self.chosen_modules.append("inverse_" + mid)
        self.m = self.modules[mid].inverse(s)
        return self.m
    
    def dist_angle(self, a1, a2): return min(abs(a1 - a2), 2 - abs(a1 - a2))
    
    def perceive(self, s, m_demo=None, j_demo=False):
        s = self.sensory_primitive(s)
        if not hasattr(self, "m"):
            return False
        ms = self.set_ms(self.m, s)
        self.update_sensorimotor_models(ms)
        if self.mid_control is not None:
            self.modules[self.mid_control].update_im(self.modules[self.mid_control].get_m(ms), self.modules[self.mid_control].get_s(ms))
        self.t = self.t + 1
        
        for mid in self.modules.keys():
            self.progresses_evolution[mid].append(self.modules[mid].progress())
            if not ((not self.enable_hand) and mid=="mod1"):
                self.interests_evolution[mid].append(self.modules[mid].interest())
            else:
                self.interests_evolution[mid].append(0.)
        return True

    def get_normalized_interests_evolution(self):
        data = np.transpose(np.array([self.interests_evolution[mid] for mid in ["mod1", "mod2"]]))
        data_sum = data.sum(axis=1)
        data_sum[data_sum==0.] = 1.
        return data / data_sum.reshape(data.shape[0],1)
    
    def get_normalized_interests(self):
        interests = {}
        for mid in self.modules.keys():
            if not ((not self.enable_hand) and mid=="mod1"):
                interests[mid] = self.modules[mid].interest()
            else:
                interests[mid] = 0.
            
        s = sum(interests.values())
        if s > 0:
            for mid in self.modules.keys():
                interests[mid] = interests[mid] / s
        return interests
        
