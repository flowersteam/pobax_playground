import rospy
import json
from pypot.robot import from_json #Custom Poppy Torso with AX12 grippers
from os import system
from pobax_playground.srv import *
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from pypot.creatures import PoppyTorso
from threading import RLock
from rospkg import RosPack
from os.path import join
from threading import Thread
from .idle import UpperBodyIdleMotion, HeadIdleMotion


class Torso(object):
    def __init__(self):
        self.rospack = RosPack()
        with open(join(self.rospack.get_path('pobax_playground'), 'config', 'torso_params.json')) as f:
            self.params = json.load(f)

        self.execute_rate_hz = self.params['speed']
        self.execute_rate = rospy.Rate(self.execute_rate_hz)

        self.publish_rate = rospy.Rate(self.params['publish_rate'])

        self.eef_pub_l = rospy.Publisher('/pobax_playground/torso/left_arm/end_effector_pose', PoseStamped, queue_size=1)
        self.eef_pub_r = rospy.Publisher('/pobax_playground/torso/right_arm/end_effector_pose', PoseStamped, queue_size=1)
        self.js_pub_l = rospy.Publisher('/pobax_playground/torso/left_arm/joints', JointState, queue_size=1)

        self.srv_reset = None
        self.srv_execute = None
        self.srv_set_compliant = None

        self.primitive_head = None
        self.primitive_right = None

        # Protected resources
        self.torso = None
        self.in_rest_pose = False
        self.robot_lock = RLock()

    def go_to_rest(self, slow):
        with self.robot_lock:
            duration = 1 if slow else 0.25
            self.set_torque_limits(60)
            #self.torso.goto_position({'l_shoulder_y': 0, 'l_shoulder_x': 0, 'l_elbow_y': 0, 'l_claw_x': 0}, duration)
            self.go_to([0,0,0,0,0,10,60,60,0,0,0,0,0,0,0], duration)
            self.torso.motors[4].compliant = True  # This motor overheats a lot
            self.in_rest_pose = True
            self.set_torque_limits()

    def go_to(self, motors, duration):
        motors_dict = dict(zip([m.name for m in self.torso.motors], motors))
        self.torso.goto_position(motors_dict, duration)
        rospy.sleep(duration)

    def set_torque_limits(self, value=None):
        for m in self.torso.ext_l_arm:
            m.torque_limit = self.params['torques'] if value is None else value

    def run(self, dummy=False):
        rospy.loginfo("Torso is connecting to the robot...")
        self.torso = PoppyTorso(use_http=True, simulator='poppy-simu' if dummy else None,config=join(self.rospack.get_path('pobax_playground'), 'config', 'torso.json'))
        if self.torso == None:
            rospy.logerr("Torso failed to init: {}".format(e))
            return None
        
        self.primitive_head = HeadIdleMotion(self.torso, 15)
        self.primitive_right = UpperBodyIdleMotion(self.torso, 15)
        self.go_to([0,0,0,0,0,10,60,60,0,0,0,0,0,0,0], 4)
        self.primitive_head.start()
        self.primitive_right.start()

        try:
            self.set_torque_limits()
            self.torso.compliant = False
            self.go_to_rest(True)

            self.srv_reset = rospy.Service('/pobax_playground/torso/reset', Reset, self._cb_reset)
            self.srv_execute = rospy.Service('/pobax_playground/torso/execute', ExecuteTorsoTrajectory, self._cb_execute)
            self.srv_set_compliant = rospy.Service('/pobax_playground/torso/set_compliant', SetTorsoCompliant, self._cb_set_compliant)
            self.srv_set_safe = rospy.Service('pobax_playground/torso/set_safe', SetTorsoSafe,self._cb_set_safe)

            rospy.loginfo("Torso is ready to execute trajectories at {} Hz ".format(self.execute_rate_hz))

            while not rospy.is_shutdown():
                self.publish_eef(self.torso.l_arm_chain.end_effector, self.eef_pub_l)
                self.publish_eef(self.torso.r_arm_chain.end_effector, self.eef_pub_r)
                self.publish_js()
                self.publish_rate.sleep()
        finally:
            self.primitive_right.stop()
            self.primitive_head.stop()
            self.torso.close()

    def publish_eef(self, eef_pose, publisher):
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'torso_base'
        pose.pose.position.x = eef_pose[0]
        pose.pose.position.y = eef_pose[1]
        pose.pose.position.z = eef_pose[2]
        publisher.publish(pose)

    def publish_js(self):
        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = [m.name for m in self.torso.ext_l_arm]
        js.position = [m.present_position for m in self.torso.ext_l_arm]
        js.velocity = [m.present_speed for m in self.torso.ext_l_arm]
        js.effort = [m.present_load for m in self.torso.ext_l_arm]
        self.js_pub_l.publish(js)

    def _cb_execute(self, request):
        # TODO Action server
        thread = Thread(target=self.execute, args=[request.torso_trajectory])
        thread.daemon = True
        thread.start()
        return ExecuteTorsoTrajectoryResponse()      

    def execute(self, trajectory):
        with self.robot_lock:
            rospy.loginfo("Executing Torso trajectory with {} points...".format(len(trajectory.points)))
            if not self.in_rest_pose:
                self.go_to_rest(False)
            for point in trajectory.points:
                if rospy.is_shutdown():
                    break
                self.torso.goto_position(dict(zip(trajectory.joint_names, point.positions)), 1.05/self.execute_rate_hz)
                self.execute_rate.sleep()
            self.in_rest_pose = False
            rospy.loginfo("Trajectory ended!")

    def _cb_set_compliant(self, request):
        with self.robot_lock:
            self.left_arm_compliant(request.compliant)
        return SetTorsoCompliantResponse()

    def left_arm_compliant(self, compliant):
        rospy.loginfo("Torso left arm now {}".format('compliant' if compliant else 'rigid'))
        for m in self.torso.l_arm:
            m.compliant = compliant

    def go_to_safe(self,duration=1):
        if not self.in_rest_pose:
            self.go_to_rest(False)
        self.primitive_head.pause()
        self.primitive_right.stop() #dirty workaround to pause this primitive
        self.go_to([30,-10,0,0,0,10,0,55,0,80,20,0,-40,0,0],duration)
        self.primitive_head.resume()
        self.primitive_right = UpperBodyIdleMotion(self.torso, 15) #again, dirty

    def _cb_reset(self, request):
        rospy.loginfo("Resetting Torso{}...".format(" in slow mode" if request.slow else ""))

        if request.slow:
            with self.robot_lock:
                self.left_arm_compliant(False)
                self.go_to_rest(True)
        else:
            with self.robot_lock:
                self.left_arm_compliant(False)
                self.go_to_rest(False)
        return ResetResponse()

    def _cb_set_safe(self, request):
        rospy.loginfo("Setting Torso in safe pose")
        with self.robot_lock:
                self.left_arm_compliant(False)
                self.go_to_safe()
        return SetTorsoSafeResponse()   
  