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

'''
class Torso_controller(object):
    self.angle_limits_ext_subl l_arm = [(-50,20),(-10,50),(-30,55),(-30,20),(0,80)]

    def __init__(self):
        self.rospack = RosPack()
        robot_config = join(self.rospack.get_path('pobax_playground'), 'config', 'torso.json')
        self.torso = from_json(robot_config)
        self.torso.compliant = False
        self.go_to_rest(slow=True)
        self.set_torque_max(20)
    
    def set_torque_max(self, torque_max=100):
        for m in self.torso.motors:
            m.torque_limit = torque_max

    def go_to_rest(self, slow=True):
        duration = 2 if slow else 0.25
        self.go_to([0 for i in range(len(torso.motors))], duration)
        self.in_rest_pose = True

    def go_to(self, motors, duration):
        motors_dict = dict(zip([m.name for m in self.torso.motors], motors))
        self.torso.goto_position(motors_dict, duration)
        rospy.sleep(duration)

    

    def close(self):
        self.torso.close()
'''
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
    def __init__(self):
        self.services = {'command': {'name': '/pobax_playground/baxter/command', 'type': BaxterCommand},
                         'get_result': {'name': '/pobax_playground/baxter/get_result', 'type': BaxterResult}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def send_command(self,cmd):
        try:
            resp1 = self.services['command']['call'](cmd)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

    def replace(self):
        rospy.loginfo('replacing culbuto using baxter...')
        # Detects when a new command can be issued by checking when a new result is produced
        self.old_result_id = self.services['get_result']['call']().result.status.goal_id.id      
        self.services['command']['call']('g1')
        for i in range(100):
            result = self.services['get_result']['call']().result
            result_id = result.status.goal_id.id
            if result_id != self.old_result_id: # If previous command finished
                self.old_result_id = result_id
                result_status = result.status.status
                break
            rospy.sleep(0.5)
        if result_status != 3:
            raw_input("Baxter could not grasp culbuto, please help him and press enter")
        self.services['command']['call']('p1')
        for i in range(100):
            result = self.services['get_result']['call']().result
            result_id = result.status.goal_id.id
            if result_id != self.old_result_id: # If previous command finished
                self.old_result_id = result_id
                result_status = result.status.status
                break
            rospy.sleep(0.5)
        if result_status != 3:
            raw_input("Baxter could not place culbuto, please help him and press enter")
        rospy.loginfo('replaced !')

    def reset(self,blocking=True):
        rospy.loginfo('Resetting Baxter')
        self.old_result_id = self.services['get_result']['call']().result.status.goal_id.id
        self.services['command']['call']('r')
        if blocking:
            for i in range(100):
                result_id = self.services['get_result']['call']().result.status.goal_id.id
                if result_id != self.old_result_id: # If previous command finished
                    self.old_result_id = result_id
                    break
                rospy.sleep(0.5)


class Learning_controller(object):

    def __init__(self):
        self.services = {'produce': {'name': '/pobax_playground/learning/produce', 'type': Produce},
                         'perceive': {'name': '/pobax_playground/learning/perceive', 'type': Perceive}}

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


class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'general.json')) as f:
            self.params = json.load(f)

        self.torso = Torso_controller()
        self.baxter = Baxter_controller()
        self.learning = Learning_controller()
        self.perception = Perception_controller()
        rospy.loginfo('Controller fully started!')



    def run(self):
        rospy.loginfo("controller node up and running")
        nb_iterations = rospy.get_param('/pobax_playground/iterations')
        self.iteration = 0
        try:
            while not rospy.is_shutdown() and self.iteration < nb_iterations:
                rospy.logwarn("#### Iteration {}/{}".format(self.iteration, nb_iterations))
                trajectory = self.learning.produce().torso_trajectory
                self.torso.execute_trajectory(trajectory)
                recording = self.perception.record(nb_points=self.params['nb_points'])
                recording.demo.torso_demonstration = JointTrajectory()
                #checks wether baxter must replace culbuto at Torso's arm reach
                culbuto = self.perception.get().state.culbuto_1
                if culbuto.pose.position.x < self.params['baxter_grasp_bound_x']:
                    self.torso.set_safe_pose() #should be a blocking call
                    self.baxter.replace()
                    self.baxter.reset(blocking=False)
                    self.torso.reset(True)
                    
                else:
                    self.torso.reset(True)
                self.learning.perceive(recording.demo)
                self.iteration += 1
        finally:
            pass
            #self.torso.close()


if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
