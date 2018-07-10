#!/usr/bin/env python
import rospy
import json
from os.path import join
from pypot.robot import from_json  # Custom Poppy Torso with AX12 grippers
from rospkg import RosPack
import os
from thr_interaction_controller.srv import *
from pobax_playground.srv import *
from pobax_playground.msg import *
from std_msgs.msg import UInt8
from trajectory_msgs.msg import JointTrajectory
from geometry_msgs.msg import Point
from pobax_playground.tools.audio_help import AudioHelp
import numpy as np
import actionlib
import pickle
import pyaudio  
import wave
from tf.transformations import euler_from_quaternion
import time

class Perception_controller(object):
    def __init__(self):
        self.services = {'record': {'name': '/pobax_playground/perception/record', 'type': Record},
                         'get': {'name': '/pobax_playground/perception/get', 'type': GetSensorialState}}
        for service_name, service in self.services.items():
            rospy.loginfo("Perception controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)

        # Init Recording action server client for non blocking recording
        self.record_server_name = '/pobax_playground/perception/record_server'
        self.client = actionlib.SimpleActionClient(self.record_server_name, RecordAction)
        self.client.wait_for_server()

    # Blocking record
    def record(self, nb_points):
        call = self.services['record']['call']
        return call(RecordRequest(nb_points=UInt8(data=nb_points))).sensorial_trajectory

    def get(self):
        call = self.services['get']['call']
        return call(GetSensorialStateRequest())

    # Starts non-blocking recording
    def start_recording(self,nb_points):
        self.client.send_goal(RecordGoal(nb_points=UInt8(data=nb_points)))

    def stop_recording(self):
        self.client.cancel_goal()
        self.client.wait_for_result()
        return self.client.get_result().sensorial_trajectory

class Torso_controller(object):
    def __init__(self):
        self.services = {'exec_torso': {'name': '/pobax_playground/torso/execute', 'type': ExecuteTorsoTrajectory},
                         'reset_torso': {'name': '/pobax_playground/torso/reset', 'type': Reset},
                         'safe_torso': {'name': '/pobax_playground/torso/set_safe', 'type': SetTorsoSafe}}
        for service_name, service in self.services.items():
            rospy.loginfo("Torso controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def reset(self, slow=False):
        call = self.services['reset_torso']['call']
        return call(ResetRequest(slow=slow))

    def execute_trajectory(self, trajectory):
        call = self.services['exec_torso']['call']
        return call(ExecuteTorsoTrajectoryRequest(torso_trajectory=trajectory))

    def set_safe_pose(self):
        return self.services['safe_torso']['call']()

class Baxter_controller(object):
    def __init__(self,result_rate):
        self.services = {'command': {'name': '/pobax_playground/baxter/command', 'type': BaxterCommand},
                         'get_result': {'name': '/pobax_playground/baxter/get_result', 'type': BaxterResult}}
        for service_name, service in self.services.items():
            rospy.loginfo("Baxter controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        self.rate = rospy.Rate(result_rate)

        self.audio_help = AudioHelp()

    def send_command(self,cmd):
        try:
            resp1 = self.services['command']['call'](cmd)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    #wait for newly published result and returns status
    def  wait_for_result(self):
        rospy.loginfo("Waiting for baxter to finish current action ...")
        while not rospy.is_shutdown():
            result = self.services['get_result']['call']().result
            result_id = result.status.goal_id.id
            if result_id != self.old_result_id: # If previous command finished
                self.old_result_id = result_id
                rospy.loginfo("Baxter's Action finished!")
                return result.status.status
            self.rate.sleep()

    def replace(self):
        rospy.loginfo('replacing culbuto using baxter...')
        # Detects when a new command can be issued by checking when a new result is produced
        self.old_result_id = self.services['get_result']['call']().result.status.goal_id.id      
        self.services['command']['call']('g1')
        result_status = self.wait_for_result()      
        if result_status != 3:
            self.audio_help.play('grasp')
            raw_input("Baxter could not grasp culbuto, please help him and press enter")

        self.services['command']['call']('p1')
        result_status = self.wait_for_result()
        if result_status != 3:
            self.audio_help.play('place')
            raw_input("Baxter could not place culbuto, please help him and press enter")

        rospy.loginfo('replaced !')

    def reset(self,blocking=True):
        rospy.loginfo('Resetting Baxter')
        self.old_result_id = self.services['get_result']['call']().result.status.goal_id.id
        self.services['command']['call']('r')
        if blocking:
            result_status = self.wait_for_result()
            if result_status != 3:
                self.audio_help.play('reset')
                raw_input("Baxter could not reset, please help him and press enter")

class Learning_controller(object):

    def __init__(self):
        self.services = {'produce': {'name': '/pobax_playground/learning/produce', 'type': Produce},
                         'perceive': {'name': '/pobax_playground/learning/perceive', 'type': Perceive},
                         'save': {'name': '/pobax_playground/learning/save', 'type': Save}}

        for service_name, service in self.services.items():
            rospy.loginfo("Learning controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def perceive(self, context, s_resp_phys, s_resp_t_sound, s_resp_b_sound):
        call = self.services['perceive']['call']
        context_msg = Point(x=context[0],y=context[1],z=context[2])
        return call(PerceiveRequest(s_context=context_msg,
                                    s_response_physical=s_resp_phys,
                                    s_response_torso_sound=s_resp_t_sound,
                                    s_response_baxter_sound=s_resp_b_sound))

    def produce(self):
        call = self.services['produce']['call']
        return call(ProduceRequest())

    def save(self):
        call = self.services['save']['call']
        return call(SaveRequest())

class Voice_controller(object):

    def __init__(self):
       self.services = {'exec_analyse': {'name': '/pobax_playground/voice/execute_analyse', 'type': ExecuteAnalyseVocalTrajectory},
                        'baxter_analyse': {'name': '/pobax_playground/voice/baxter_analyse', 'type': BaxterAnalyseVocalTrajectory}} 

       for service_name, service in self.services.items():
            rospy.loginfo("Voice controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def execute_analyse(self, trajectory_msg):
        request = ExecuteAnalyseVocalTrajectoryRequest(vocal_trajectory=trajectory_msg)
        response = self.services['exec_analyse']['call'](request)
        return response.torso_sound_trajectory, response.baxter_sound_trajectory,response.is_culbuto_name,\
               response.produced_name, response.raw_torso_sound

    def baxter_analyse(self, is_culbuto_touched):
        request = BaxterAnalyseVocalTrajectoryRequest(is_culbuto_touched=is_culbuto_touched)
        return self.services['baxter_analyse']['call'](request).baxter_sound_trajectory

class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'general.json')) as f:
            self.params = json.load(f)

        self.torso = Torso_controller()
        self.baxter = Baxter_controller(self.params["baxter_result_refresh_rate"])
        self.learning = Learning_controller()
        self.perception = Perception_controller()
        self.voice = Voice_controller()

        self.starting_iteration = rospy.get_param("/pobax_playground/starting_iteration")

        #Init save file for book_keeping
        self.experiment_name = rospy.get_param("/pobax_playground/experiment_name", "experiment")

        # Saved experiment files
        self.dir = join(self.rospack.get_path('pobax_playground'), 'logs')
        if not os.path.isdir(self.dir):
            os.makedirs(self.dir)
        self.experiment_bk_file = join(self.dir, self.experiment_name + '_book_keeping.pickle')

        # Sound setting
        self.audio_help = AudioHelp()

        # Quick and DIRTY TODO use params
        self.culb_orientation_delta = 0.50


        rospy.loginfo('Controller fully started!')

    def play_help_msg(self,msg_name):
        #read data 
        f = self.help_msg[msg_name]['file']
        data = f.readframes(self.chunk)
        stream = self.p.open(format = self.p.get_format_from_width(f.getsampwidth()),  
            channels = f.getnchannels(),  
            rate = f.getframerate(),  
            output = True) 
        #play stream  
        while data:  
            stream.write(data)  
            data = f.readframes(self.chunk)  
        #stop stream  
        stream.stop_stream() 
        stream.close()

    def save_bk(self, book_keeping_dict):
        with open(self.experiment_bk_file, 'w') as f:
            pickle.dump(book_keeping_dict, f)

    def load_bk(self):
        with open(self.experiment_bk_file, 'r') as f:
            bk_dict = pickle.load(f)
        return bk_dict

    # Extracts culbuto's x,y,z coordinates from PoseStamped msg
    def get_culb_coord(self, msg, orientation=False):
        coords = msg.state.culbuto_1.pose.position
        quaternion = msg.state.culbuto_1.pose.orientation
        if orientation:
            return coords.x, coords.y, coords.z, quaternion
        else:
            return coords.x, coords.y, coords.z

    # Loops while culbuto is still moving and returns the number of waiting loops
    def wait_motionless_culbuto(self):
        rospy.loginfo('Waiting for culbuto being immobile...')
        old_x,old_y,old_z = self.get_culb_coord(self.perception.get())
        nb_iter = 0
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            cur_x, cur_y, cur_z = self.get_culb_coord(self.perception.get())
            nb_iter += 1
            if (abs(cur_x-old_x) + abs(cur_y-old_y) + abs(cur_y-old_y)) < 0.0005:
                rospy.loginfo('Finished waiting, culbuto is motionless!')
                return nb_iter
            old_x,old_y,old_z = cur_x,cur_y,cur_z

    # Returns True if culbuto is outside torso's armreach, False otw
    def is_culbuto_too_far(self):
        culb_x,_,culb_z = self.get_culb_coord(self.perception.get())
        if culb_x < self.params['baxter_grasp_bound_x']:
            return True
        xs,zs = self.params['baxter_grasp_bound_line']
        a,b = np.polyfit(xs,zs,1) #get line's equation
        if culb_z > (culb_x * a + b):
            return True
        return False

    # Returns True if culbuto has been touched during torso's movement, False otw
    def is_culbuto_touched(self,s_response_physical):
        thr = 9e-06 #variance threshold
        xs = []
        ys = []
        zs = []
        for point in s_response_physical.points:
            xs.append(round(point.culbuto_1.pose.position.x,4))
            ys.append(round(point.culbuto_1.pose.position.y,4))
            zs.append(round(point.culbuto_1.pose.position.z,4))
        rospy.logwarn("X:{},Y:{},Z:{}".format(np.var(xs), np.var(ys), np.var(zs)))
        if (np.var(xs) > thr) or (np.var(ys) > thr) or (np.var(zs) > thr):
            return True
        else:
            return False

    # Checks wether culbuto is still in the playground, ask user to put it back otw
    def ensure_culbuto_safe_pos(self, culb_pose):
        culb_coord = culb_pose[0:3]
        culb_orientation = (culb_pose[3].x, culb_pose[3].y, culb_pose[3].z, culb_pose[3].w)
        if culb_coord[1] > self.params['culbuto_height_limit']:
            self.audio_help.play('away')
            raw_input("Culbuto out of playground, please set it back and press enter")
        # not working, not precise enough
        #print euler_from_quaternion(culb_orientation)
        #euler_o = np.abs(euler_from_quaternion(culb_orientation))
        #if ((euler_o[0] > self.culb_orientation_delta) or (euler_o[2] > self.culb_orientation_delta))\
        #   and ((euler_o[0] < (2.85-self.culb_orientation_delta) or euler_o[2] < (2.85-self.culb_orientation_delta))):
        #   self.audio_help.play('away')
        #   raw_input("Culbuto out of playground, please set it back and press enter")


    def run(self):
        rospy.loginfo("controller node up and running")
        nb_iterations = rospy.get_param('/pobax_playground/iterations')
        self.iteration = self.starting_iteration

        #Init Book-Keeping dict or load it if starting from previous run
        if self.iteration > 0:
            b_k = self.load_bk()
        else:
            b_k = dict()
            b_k['nb_culbuto_touched'] = 0
            b_k['nb_culbuto_pronounced'] = 0
            b_k['nb_motor_it'] = 0
            b_k['nb_sound_it'] = 0
            b_k['produced_names'] = dict()
            b_k['raw_torso_sounds'] = []

        try:
            while not rospy.is_shutdown() and self.iteration < nb_iterations:
                self.iteration += 1
                rospy.logwarn("#### Iteration {}/{}".format(self.iteration, nb_iterations))
                rospy.logwarn("Book-keeping: culb_touched= {}, culb_said={}, nb_motor: {}, nb_sound: {}".format(b_k['nb_culbuto_touched'], b_k['nb_culbuto_pronounced'], b_k['nb_motor_it'], b_k['nb_sound_it']))
                rospy.logwarn("List of torso's produced sounds: {}".format(b_k['produced_names']))
                # Init sensorial responses
                s_response_physical = None
                s_response_torso_sound = None
                s_response_baxter_sound = None
                #self.ensure_culbuto_safe_pos(self.get_culb_coord(self.perception.get(),orientation=True))
                #self.wait_motionless_culbuto()
                traj_msg = self.learning.produce()
                s_context = self.get_culb_coord(self.perception.get())
                if traj_msg.trajectory_type == "diva":
                    b_k['nb_sound_it'] += 1
                    rospy.loginfo('Controller received a vocal trajectory')
                    #print np.shape(traj_msg.vocal_trajectory.data)
                    s_response_torso_sound, s_response_baxter_sound, \
                    is_culbuto_name, produced_name, raw_torso_sound \
                    = self.voice.execute_analyse(traj_msg.vocal_trajectory)
                    #print "torso sounds:"
                    #print s_response_torso_sound
                    b_k['raw_torso_sounds'] += [raw_torso_sound]
                    if produced_name:
                        if not produced_name in b_k['produced_names']: 
                            b_k['produced_names'][produced_name] = 1
                        else:
                            b_k['produced_names'][produced_name] += 1
                    if is_culbuto_name:
                        b_k['nb_culbuto_pronounced'] += 1
                        #checks wether baxter must replace culbuto at Torso's arm reach
                        if self.is_culbuto_too_far():
                            self.torso.set_safe_pose() #should be a blocking call
                            self.perception.start_recording(self.params['nb_points'])
                            self.baxter.replace()
                            s_response_physical = self.perception.stop_recording()
                            is_culbuto_touched_test = self.is_culbuto_touched(s_response_physical)
                            if is_culbuto_touched_test:
                                rospy.logwarn("SHOULD NOT HAPPEN")
                                raw_input("WTF CULBUTO WAS TOUCHED WHILE TALKING BRUUUH")

                            self.baxter.reset(blocking=False)
                            self.torso.reset(True)
                            self.wait_motionless_culbuto()
                            self.ensure_culbuto_safe_pos(self.get_culb_coord(self.perception.get(),orientation=True))

                elif traj_msg.trajectory_type == "arm":
                    b_k['nb_motor_it'] += 1
                    rospy.loginfo('Controller received a torso trajectory')
                    self.torso.execute_trajectory(traj_msg.torso_trajectory)
                    s_response_physical = self.perception.record(nb_points=self.params['nb_points']) #blocking
                    self.torso.reset(True)
                    is_culbuto_touched = self.is_culbuto_touched(s_response_physical)
                    if is_culbuto_touched: b_k['nb_culbuto_touched'] += 1
                    s_response_baxter_sound = self.voice.baxter_analyse(is_culbuto_touched)
                else:
                    rospy.logerr('Controller received an unknown trajectory type')

                self.learning.perceive(s_context, s_response_physical, s_response_torso_sound, s_response_baxter_sound)
                if traj_msg.trajectory_type == "arm":
                    # checking if culbuto is motionless takes time, so we do not perform this check 
                    # for diva motor command (which cannot end up moving the culb)
                    self.wait_motionless_culbuto()
                    self.ensure_culbuto_safe_pos(self.get_culb_coord(self.perception.get(),orientation=True))
                # Reset culbuto periodically
                if self.iteration % self.params['reset_every'] == 0:
                    if self.is_culbuto_too_far():
                        rospy.loginfo("Periodic reset of culbuto (which is too far for torso)")
                        self.torso.set_safe_pose() #should be a blocking call
                        self.baxter.replace()
                        self.baxter.reset(blocking=False)
                        self.torso.reset(True)
                        self.wait_motionless_culbuto()
                        self.ensure_culbuto_safe_pos(self.get_culb_coord(self.perception.get(),orientation=True))
                if self.iteration % self.params['save_every'] == 0:
                    self.learning.save()
                    self.save_bk(b_k)
        finally:
            pass


if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
