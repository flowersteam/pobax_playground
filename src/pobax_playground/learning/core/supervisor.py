import numpy as np

from explauto.utils import rand_bounds, bounds_min_max, softmax_choice, prop_choice
from explauto.utils.config import make_configuration
from learning_module import LearningModule


class Supervisor(object):
    def __init__(self, config, model_babbling="random", n_motor_babbling=0, explo_noise=0.1, choice_eps=0.2, proba_imitate=0.5, tau=1.):
        
        self.config = config
        self.model_babbling = model_babbling
        self.n_motor_babbling = n_motor_babbling
        self.explo_noise = explo_noise
        self.choice_eps = choice_eps
        self.proba_imitate = proba_imitate
        self.conf = make_configuration(**config)
        
        self.t = 0
        self.modules = {}
        self.chosen_modules = []
        self.goals = []
        self.cp_evolution = {}
        self.pp_evolution = {}
        
        self.mid_control = None
        self.last_cmd = None
        
        # Define motor and sensory spaces:
        m_ndims = self.conf.m_ndims # number of motor parameters
        self.arm_n_dims = 40
        self.diva_n_dims = 28
        assert(m_ndims == self.arm_n_dims + self.diva_n_dims)

        self.m_arm = range(self.arm_n_dims)
        self.m_diva = range(self.arm_n_dims,self.arm_n_dims + self.diva_n_dims)

        self.m_space = range(m_ndims)
        self.c_dims = range(m_ndims, m_ndims+3)
        self.s_hand = range(m_ndims+3, m_ndims+33)
        self.s_culbuto_1 = range(m_ndims+33, m_ndims+63)
        self.s_self_sound = range(m_ndims+63, m_ndims+73)
        self.s_caregiver_sound = range(m_ndims+73, m_ndims+83)
      
        self.s_spaces = dict(s_hand=self.s_hand, 
                             s_culbuto_1=self.s_culbuto_1,
                             s_self_sound=self.s_self_sound, 
                             s_caregiver_sound=self.s_caregiver_sound)

        self.arm_modules = ['mod1','mod3','mod6']
        self.diva_modules = ['mod12','mod14']
        self.arm_goal_selection = 0.10

        
        # Create the learning modules:
        self.modules['mod1'] = LearningModule("mod1", self.m_arm, self.s_hand, self.conf, explo_noise=self.explo_noise)
        self.modules['mod3'] = LearningModule("mod3", self.m_arm,self.c_dims + self.s_culbuto_1, self.conf, context_mode=dict(mode='mcs', context_dims=[0,1,2], context_n_dims=3, context_sensory_bounds=[[-2.]*3,[2.]*3]), explo_noise=self.explo_noise)
        self.modules['mod6'] = LearningModule("mod6", self.m_arm, self.c_dims + self.s_caregiver_sound, self.conf, context_mode=dict(mode='mcs', context_dims=[0,1,2], context_n_dims=3, context_sensory_bounds=[[-2.]*3,[2.]*3]), explo_noise=self.explo_noise)
        
        self.modules['mod12'] = LearningModule("mod12", self.m_diva, self.c_dims + self.s_culbuto_1, self.conf, context_mode=dict(mode='mcs', context_dims=[0,1,2], context_n_dims=3, context_sensory_bounds=[[-2.]*3,[2.]*3]), explo_noise=self.explo_noise)
        #self.modules['mod13'] = LearningModule("mod13", self.m_diva, self.s_self_sound, self.conf, explo_noise=self.explo_noise)
        self.modules['mod14'] = LearningModule("mod14", self.m_diva, self.s_self_sound, self.conf, imitate="mod6", explo_noise=self.explo_noise, proba_imitate=self.proba_imitate) 

        for mid in self.modules.keys():
            self.cp_evolution[mid] = []
            self.pp_evolution[mid] = []
            
        self.count_arm = 0
        self.count_diva = 0
        
        self.mids = ["mod"+ str(i) for i in range(1, 15) if "mod"+ str(i) in self.modules.keys()]

        #Book-keeping
        self.actions = []
        self.observations = []

    
    def mid2motor_space(self, mid):
        if mid in ["mod"+ str(i) for i in range(1, 8)]:
            return "arm"
        else:
            return "diva"
    
    def save(self):
        sm_data = {}
        im_data = {}
        for mid in self.modules.keys():
            sm_data[mid] = self.modules[mid].sensorimotor_model.save()
            im_data[mid] = self.modules[mid].interest_model.save()             
        return {
                "actions": self.actions,
                "observations": self.observations,
                "sm_data":sm_data,
                #"im_data":im_data,
                "goals":self.goals,
                "chosen_modules":self.chosen_modules,
                #"cp_evolution":self.cp_evolution,
                #"pp_evolution":self.pp_evolution,
                #"imitated_sounds":self.modules['mod14'].imitated_sounds
                }

    def forward(self, data, iteration):
        if iteration > len(data["chosen_modules"]):
            print "\nWARNING: asked to restart from iteration", iteration, "but only", len(data["chosen_modules"]), "are available. Restarting from iteration", len(data["chosen_modules"]), "..."
            iteration = len(data["chosen_modules"])
        if iteration < 0:
            iteration = len(data["chosen_modules"])
        self.chosen_modules = data["chosen_modules"][:iteration]
        self.observations = data["observations"][:iteration]
        self.actions = data["actions"][:iteration]
        self.goals = data["goals"][:iteration]
        #self.pp_evolution = data["pp_evolution"]
        #self.interests_evolution = data["interests_evolution"]
        #for mid in self.modules.keys():
        #    self.progresses_evolution[mid] = self.progresses_evolution[mid][:iteration]
        #    self.interests_evolution[mid] = self.interests_evolution[mid][:iteration]
        self.t = iteration
        if iteration > 0:
            for mid in self.modules.keys():
                self.modules[mid].sensorimotor_model.forward(data["sm_data"][mid], iteration)
                    
                #self.modules[mid].interest_model.forward(data["im_data"][mid], self.chosen_modules.count(mid), self.pp_evolution[mid][-1], self.interests_evolution[mid][-1])


        
    def choose_babbling_module(self):
        interests = {}
        for mid in self.modules.keys():
            interests[mid] = self.modules[mid].interest()
        if self.model_babbling == 'random':
            #mid = np.random.choice(interests.keys())
            if np.random.random() < self.arm_goal_selection:
                mid = np.random.choice(self.arm_modules)
            else:
                mid = np.random.choice(self.diva_modules)
        elif self.model_babbling == 'hand_object_sound':
            if np.random.random() < 1. / 3.:
                mid = 'mod1'
            elif np.random.random() < 1. / 2.:
                mid = np.random.choice(['mod2', 'mod3', 'mod4', 'mod5', 'mod10', 'mod11', 'mod12'])
            else:
                mid = np.random.choice(['mod6', 'mod13', 'mod14'])
        elif self.model_babbling == 'object_sound':
            if np.random.random() < 1. / 2.:
                mid = np.random.choice(['mod1', 'mod2', 'mod3', 'mod4', 'mod5', 'mod10', 'mod11', 'mod12'])
            else:
                mid = np.random.choice(['mod6', 'mod13', 'mod14'])
        elif self.model_babbling == 'greedy':
            if np.random.random() < self.choice_eps:
                mid = np.random.choice(interests.keys())
            else:
                mid = max(interests, key=interests.get)
        elif self.model_babbling == 'softmax':
            temperature = self.choice_eps
            w = interests.values()
            mid = self.modules.keys()[softmax_choice(w, temperature)]
        
        elif self.model_babbling == 'prop':
            w = interests.values()
            mid = self.modules.keys()[prop_choice(w, eps=self.choice_eps)]
        
        self.chosen_modules.append(int(mid[3:]))
        return mid
              
        
    def eval_mode(self): 
        self.sm_modes = {}
        for mod in self.modules.values():
            self.sm_modes[mod.mid] = mod.sensorimotor_model.mode
            mod.sensorimotor_model.mode = 'exploit'
                
    def learning_mode(self): 
        for mod in self.modules.values():
            mod.sensorimotor_model.mode = self.sm_modes[mod.mid]
                
    def motor_primitive(self, m): return m
    def sensory_primitive(self, s): return s
    def get_m(self, ms): return ms[self.conf.m_dims]
    def get_s(self, ms): return ms[self.conf.s_dims]
    
    def motor_babbling(self, arm=False, audio=False):
        self.m = rand_bounds(self.conf.m_bounds)[0]
        if arm:
            r = 1.
            self.last_cmd = "arm"
        elif audio:
            r = 0.
            self.last_cmd = "diva"
        else:
            r = np.random.random()
        module = None
        if r > self.arm_goal_selection:
            self.m[:self.arm_n_dims] = 0.
            self.last_cmd = "diva"
            module = "diva_babbling"
        else:
            self.m[self.arm_n_dims:] = 0.
            self.last_cmd = "arm"
            module = "arm_babbling"
        self.chosen_modules.append(module)
        return self.m, self.last_cmd
    
    def set_ms(self, m, s): return np.array(list(m) + list(s))
            
    def update_sensorimotor_models(self, ms):
        for mid in self.modules.keys():
            if self.last_cmd == self.mid2motor_space(mid):
                self.modules[mid].update_sm(self.modules[mid].get_m(ms), self.modules[mid].get_s(ms))
        
    def produce(self, context):
        if self.t < self.n_motor_babbling:
            self.mid_control = None
            return self.motor_babbling()
        else:
            mid = self.choose_babbling_module()
            self.mid_control = mid
            
            mid_c = self.modules[mid].get_c(context) if self.modules[mid].context_mode else None
            
            if self.modules[mid].imitate is not None:
                m = self.modules[mid].produce(context=mid_c, imitate_sm=self.modules[self.modules[mid].imitate].sm.model.imodel.fmodel.dataset)
            else:                
                m = self.modules[mid].produce(context=mid_c)
                
            if self.mid2motor_space(mid) == "arm":
                self.last_cmd = "arm"
                self.m = list(m) + [0.]*self.diva_n_dims
                self.count_arm += 1
            else:
                self.last_cmd = "diva"
                self.m = [0.]*self.arm_n_dims + list(m)
                self.count_diva += 1
            if mid is not None:
                self.goals.append(self.modules[mid].x)
            return self.m, self.last_cmd
    
    def perceive(self, s):
        s = self.sensory_primitive(s)
        if not hasattr(self, "m"):
            return False
        ms = self.set_ms(self.m, s)
        self.observations += [s]
        self.actions += [self.m]
        self.update_sensorimotor_models(ms)
        if self.mid_control is not None:
            self.modules[self.mid_control].update_im(self.modules[self.mid_control].get_m(ms), self.modules[self.mid_control].get_s(ms))
        self.t = self.t + 1
        
        if self.t % 100 == 0:
            for mid in self.modules.keys():
                self.cp_evolution[mid].append(self.modules[mid].interest_model.current_competence_progress)
                self.pp_evolution[mid].append(self.modules[mid].interest_model.current_prediction_progress)           
            
        if self.t % 1000 == 0:
            self.print_stats() 
        return True
        
    def print_stats(self):
        print "\n----------------\nAgent Statistics\n----------------\n"
        print "#Iterations:", self.t
        print
        for mid in self.mids:
            print "# Chosen module", mid, ":", self.chosen_modules.count(int(mid[3:]))
        print
        for mid in self.mids:
            print "Competence progress of", mid, ": " if mid in ["mod10", "mod11", "mod12", "mod13", "mod14"] else " : ", self.modules[mid].interest_model.current_competence_progress
        print
        for mid in self.mids:
            print "Prediction progress of", mid, ": " if mid in ["mod10", "mod11", "mod12", "mod13", "mod14"] else " : ", self.modules[mid].interest_model.current_prediction_progress
        print "#Arm trials", self.count_arm
        print "#Vocal trials", self.count_diva
        print
        
        
        
