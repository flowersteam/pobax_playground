#!/usr/bin/env python
import rospy
import json
from os.path import join
from pypot.robot import from_json  # Custom Poppy Torso with AX12 grippers
from rospkg import RosPack
import os
from thr_interaction_controller.srv import *
from pobax_playground.srv import *

'''
class Torso_controller(object):
    self.angle_limits_l_arm = [(-50,20),(-10,50),(-30,55),(-30,20),(0,80)]

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

    def go_to_safe(self,duration=2):
        #todo check if in rest before
        if not self.in_rest_pose:
            print("go_to_safe position failed, torso wasn't in rest pose")
            return
        go_to([30,-20,0,0,0,20,0,70,0,0,20,0,-40,0,0],duration)

    def close(self):
        self.torso.close()
'''
class Perception_controller(object):
    def __init__(self):
        self.services = {'record': {'name': '/pobax_playground/perception/record', 'type': Record}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'perception.json')) as f:
            self.params = json.load(f)

    def record(self, human_demo, nb_points):
        call = self.services['record']['call']
        return call(RecordRequest(nb_points=UInt8(data=nb_points)))

class Torso_controller(object):
    def __init__(self):
        self.services = {'exec_torso': {'name': '/pobax_playground/torso/execute', 'type': ExecuteTorsoTrajectory},
                         'reset_torso': {'name': '/pobax_playground/torso/reset', 'type': Reset}}
        for service_name, service in self.services.items():
            rospy.loginfo("Controller is waiting service {}...".format(service['name']))
            rospy.wait_for_service(service['name'])
            service['call'] = rospy.ServiceProxy(service['name'], service['type'])

    def reset(self, slow):
        call = self.services['reset_torso']['call']
        return call(ResetRequest(slow=slow))

    def execute_trajectory(self, trajectory):
        call = self.services['exec_torso']['call']
        return call(ExecuteTorsoTrajectoryRequest(torso_trajectory=trajectory))

class Baxter_controller(object):

    def __init__(self):
        rospy.wait_for_service('baxter_command')
        self.baxter_command = rospy.ServiceProxy('baxter_command', BaxterCommand)

    def send_command(self,cmd):
        try:
            resp1 = self.baxter_command(cmd)
            print resp1
        except rospy.ServiceException, e:
            print "Service call failed: %s"%e

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
        rospy.loginfo('Controller fully started!')



    def run(self):
        #self.torso.go_to_rest()
        print "controller node up and running"
        nb_iterations = rospy.get_param('/pobax_playground/iterations')
        self.iteration = 0
        try:
            while not rospy.is_shutdown() and self.iteration < nb_iterations:
                rospy.logwarn("#### Iteration {}/{}".format(self.iteration, nb_iterations))
                #trajectory = self.learning.produce(skill_to_demonstrate=self.demonstrate).torso_trajectory
                
                #trajectory = self.learning.produce().torso_trajectory
                #print "trajectory produced" 
                #print trajectory
                #self.torso.set_torque_max(15)
                #self.recorder.record(task, method, trial, iteration)
                #self.torso.execute_trajectory(trajectory)  # TODO: blocking, non-blocking, action server?
                #recording = self.perception.record(human_demo=False, nb_points=self.params['nb_points'])
                #recording.demo.torso_demonstration = JointTrajectory()
                #self.torso.set_torque_max(80)
                #self.learning.perceive("Calling Percieve, TODO implement demonstration")
                rospy.sleep(5)
                print("test")
                self.baxter.send_command("r")
                self.iteration += 1
        finally:
            pass
            #self.torso.close()


if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
