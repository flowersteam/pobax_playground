#!/usr/bin/env python
import rospy
import json
from os.path import join
from pypot.robot import from_json  # Custom Poppy Torso with AX12 grippers
from rospkg import RosPack

class Torso(object):
    def __init__(self):
        self.rospack = RosPack()
        robot_config = join(self.rospack.get_path('pobax_playground'), 'config', 'torso.json')
        self.torso = from_json(robot_config)
        self.torso.compliant = False
    
    def set_torque_max(self, torque_max=100):
        for m in self.torso.motors:
            m.torque_limit = torque_max

    def go_to_rest(self, slow=True):
        duration = 2 if slow else 0.25
        #self.torso.goto_position({'l_shoulder_y': 13, 'l_shoulder_x': 20, 'l_elbow_y': -25}, duration)
        #rospy.sleep(duration)
        self.go_to([0, 0, 0, 0, 20, 0, 0, 0, 0, 0, 0, 0, 0], duration)
        self.in_rest_pose = True

    def go_to(self, motors, duration):
        motors_dict = dict(zip([m.name for m in self.torso.motors], motors))
        self.torso.goto_position(motors_dict, duration)
        rospy.sleep(duration)

    def close(self):
        self.torso.close()


class Controller(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'general.json')) as f:
            self.params = json.load(f)

        self.torso = Torso()
        rospy.loginfo('Controller fully started!')



    def run(self):
        self.torso.go_to_rest()
        try:
            while not rospy.is_shutdown():
                #trajectory = self.learning.produce(skill_to_demonstrate=self.demonstrate).torso_trajectory
                self.torso.set_torque_max(15)
                #self.recorder.record(task, method, trial, iteration)
                #self.torso.execute_trajectory(trajectory)  # TODO: blocking, non-blocking, action server?
                #recording = self.perception.record(human_demo=False, nb_points=self.params['nb_points'])
                #recording.demo.torso_demonstration = JointTrajectory()
                #self.torso.set_torque_max(80)
                #return self.learning.perceive(recording.demo)
                rospy.sleep(1)
        finally:
            self.torso.close()
        

if __name__ == '__main__':
    rospy.init_node("controller")
    Controller().run()
