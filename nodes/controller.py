#!/usr/bin/env python
import rospy
import json
from os.path import join
from pypot.robot import from_json  # Custom Poppy Torso with AX12 grippers
from rospkg import RosPack
import os
from thr_interaction_controller.srv import *
from pobax_playground.srv import *
from std_msgs.msg import UInt8
from trajectory_msgs.msg import JointTrajectory
import numpy as np

class Perception_controller(object):
    def __init__(self):
        self.services = {'record': {'name': '/pobax_playground/perception/record', 'type': Record},
                         'get': {'name': '/pobax_playground/perception/get', 'type': GetSensorialState}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)

    def record(self, nb_points):
        call = self.services['record']['call']
        return call(RecordRequest(nb_points=UInt8(data=nb_points)))

    def get(self):
        call = self.services['get']['call']
        return call(GetSensorialStateRequest())

class Torso_controller(object):
    def __init__(self):
        self.services = {'exec_torso': {'name': '/pobax_playground/torso/execute', 'type': ExecuteTorsoTrajectory},
                         'reset_torso': {'name': '/pobax_playground/torso/reset', 'type': Reset},
                         'safe_torso': {'name': '/pobax_playground/torso/set_safe', 'type': SetTorsoSafe}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
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
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
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
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def perceive(self, demonstration):
        call = self.services['perceive']['call']
        return call(PerceiveRequest(demo=demonstration))

    def produce(self):
        call = self.services['produce']['call']
        return call(ProduceRequest())

    def save(self):
        call = self.services['save']['call']
        return call(SaveRequest())


class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'general.json')) as f:
            self.params = json.load(f)

        self.torso = Torso_controller()
        self.baxter = Baxter_controller(self.params["baxter_result_refresh_rate"])
        self.learning = Learning_controller()
        self.perception = Perception_controller()

        self.starting_iteration = rospy.get_param("/pobax_playground/starting_iteration")
        rospy.loginfo('Controller fully started!')

    # Returns True if culbuto is outside torso's armreach, False otw
    def is_culbuto_too_far(self):
        culbuto = self.perception.get().state.culbuto_1
        if culbuto.pose.position.x < self.params['baxter_grasp_bound_x']:
            return True
        xs,zs = self.params['baxter_grasp_bound_line']
        a,b = np.polyfit(xs,zs,1) #get line's equation
        if culbuto.pose.position.z > (culbuto.pose.position.x * a + b):
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
                trajectory = self.learning.produce().torso_trajectory
                self.torso.execute_trajectory(trajectory)
                recording = self.perception.record(nb_points=self.params['nb_points'])
                recording.demo.torso_demonstration = JointTrajectory()
                #checks wether baxter must replace culbuto at Torso's arm reach
                culbuto = self.perception.get().state.culbuto_1
                if self.is_culbuto_too_far():
                    self.torso.set_safe_pose() #should be a blocking call
                    self.baxter.replace()
                    self.baxter.reset(blocking=False)
                    self.torso.reset(True)
                    
                else:
                    self.torso.reset(True)
                self.learning.perceive(recording.demo)
                if self.iteration % self.params['save_every'] == 0:
                    self.learning.save()
        finally:
            pass
            #self.torso.close()


if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
