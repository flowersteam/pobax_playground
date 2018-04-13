
import os
import pickle
import matplotlib.pyplot as plt
import numpy as np
import time
import datetime

from core.supervisor import Supervisor




class Learning(object):
    def __init__(self, config, sensory_state_size, n_motor_babbling=0, explo_noise=0.1, choice_eps=0.2, enable_hand=True, normalize_interests=True):
        self.config = config
        self.sensory_state_size = sensory_state_size
        self.n_motor_babbling = n_motor_babbling
        self.explo_noise = explo_noise
        self.choice_eps = choice_eps
        self.enable_hand = enable_hand
        self.normalize_interests = normalize_interests
        self.agent = None
        
    def produce(self, context):
        return self.agent.produce(context)
            
    def perceive(self, s):
        # Perception of environment when m was produced
        assert len(s) == self.sensory_state_size, len(s)
        return self.agent.perceive(s)
            
    def get_iterations(self): return self.agent.t
    def get_normalized_interests(self): return self.agent.get_normalized_interests()    
    def get_normalized_interests_evolution(self): return self.agent.get_normalized_interests_evolution()
    def get_last_focus(self): return self.agent.get_last_focus()
    def get_space_names(self): return self.agent.get_space_names()
    def motor_babbling(self): return self.agent.motor_babbling()
    
    def get_data_from_file(self, file_path):
        with open(file_path, 'r') as f:
            data = pickle.load(f)
        return data
                
    def save(self, file_path):        
        data = self.agent.save() 
        with open(file_path, 'w') as f:
            pickle.dump(data, f)
    
    def start(self):
        self.agent = Supervisor(self.config,
                                sensory_state_size=self.sensory_state_size, 
                                n_motor_babbling=self.n_motor_babbling, 
                                explo_noise=self.explo_noise, 
                                choice_eps=self.choice_eps,
                                enable_hand=self.enable_hand,
                                normalize_interests=self.normalize_interests)
        
    def restart_from_end_of_file(self, file_path):
        data = self.get_data_from_file(file_path)
        self.start()
        nb_iter = len(data["chosen_modules"])
        self.agent.forward(data, len(data["chosen_modules"]))
        return nb_iter
    
    def restart_from_file(self, file_path, iteration):
        data = self.get_data_from_file(file_path)
        self.start()
        self.agent.forward(data, iteration)

    def plot(self):
        fig, ax = plt.subplots()
        ax.plot(self.get_normalized_interests_evolution(), lw=2)
        ax.legend(["Hand", "Joystick_1", "Joystick_2", "Ergo", "Ball", "Light", "Sound"], ncol=3)
        ax.set_xlabel('Time steps', fontsize=20)
        ax.set_ylabel('Learning progress', fontsize=20)
        plt.show(block=True)
        