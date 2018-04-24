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
import numpy as np
import actionlib

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
            raw_input("Baxter could not grasp culbuto, please help him and press enter")

        self.services['command']['call']('p1')
        result_status = self.wait_for_result()
        
        rospy.loginfo('replaced !')

    def reset(self,blocking=True):
        rospy.loginfo('Resetting Baxter')
        self.old_result_id = self.services['get_result']['call']().result.status.goal_id.id
        self.services['command']['call']('r')
        if blocking:
            result_status = self.wait_for_result()
            if result_status != 3:
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
        return response.torso_sound_trajectory, response.baxter_sound_trajectory, response.is_culbuto_name

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
        rospy.loginfo('Controller fully started!')

    # Extracts culbuto's x,y,z coordinates from PoseStamped msg
    def get_culb_coord(self, msg):
        coords = msg.state.culbuto_1.pose.position
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
            if (abs(cur_x-old_x) + abs(cur_y-old_y) + abs(cur_y-old_y)) < 0.001:
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




    def run(self):
        rospy.loginfo("controller node up and running")
        nb_iterations = rospy.get_param('/pobax_playground/iterations')
        self.iteration = self.starting_iteration
        try:
            while not rospy.is_shutdown() and self.iteration < nb_iterations:
                self.iteration += 1
                rospy.logwarn("#### Iteration {}/{}".format(self.iteration, nb_iterations))
                # Init sensorial responses
                s_response_physical = None
                s_response_torso_sound = None
                s_response_baxter_sound = None

                traj_msg = self.learning.produce()

                if traj_msg.trajectory_type == "diva":
                    rospy.loginfo('Controller received a vocal trajectory')
                    print np.shape(traj_msg.vocal_trajectory.data)
                    self.wait_motionless_culbuto()
                    s_response_torso_sound, s_response_baxter_sound, is_culbuto_name = self.voice.execute_analyse(traj_msg.vocal_trajectory)
                    #print "torso sounds:"
                    #print s_response_torso_sound
                    self.wait_motionless_culbuto() #blocking
                    if is_culbuto_name:
                        #checks wether baxter must replace culbuto at Torso's arm reach
                        if self.is_culbuto_too_far():
                            self.torso.set_safe_pose() #should be a blocking call
                            self.perception.start_recording(self.params['nb_points'])
                            self.baxter.replace()
                            s_response_physical = self.perception.stop_recording()
                            self.baxter.reset(blocking=False)
                            self.torso.reset(True)
                elif traj_msg.trajectory_type == "arm":
                    rospy.loginfo('Controller received a torso trajectory')
                    is_culbuto_touched = False
                    self.torso.execute_trajectory(traj_msg.torso_trajectory)
                    s_response_physical = self.perception.record(nb_points=self.params['nb_points']) #blocking
                    self.torso.reset(True)
                    nb_waiting_iterations = self.wait_motionless_culbuto() #blocking
                    if nb_waiting_iterations >= 2: # Culbuto was touched
                        is_culbuto_touched = True
                    s_response_baxter_sound = self.voice.baxter_analyse(is_culbuto_touched)
                else:
                    rospy.logerr('Controller received an unknown trajectory type')
                s_context = self.get_culb_coord(self.perception.get())
                self.learning.perceive(s_context, s_response_physical, s_response_torso_sound, s_response_baxter_sound)
                if self.iteration % self.params['save_every'] == 0:
                    self.learning.save()
        finally:
            pass


if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
