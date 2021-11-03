#!/usr/bin/env python

import rospy
import numpy as np
# import ml_esc
# import lowpass
from std_msgs.msg import Float64, Float32, Int16
from sensor_msgs.msg import JointState
from math import pi, sin, cos, atan
from numpy import sign, clip, random, round

def rem2pi(x):
    return x - 2*pi*np.round(x/(2*pi))

class InertiaWheelPendulum(object):

    def __init__(
        self, m1=1.423, m2=0.577, lc1=0.17/3.0, 
        l1=0.17, I1=0.0, I2=0.00289, b1=0.00, b2=0.00,
        gamma_1=6.0, gamma_2=20.0, kp=0.001, kv=0.1
    ):
        self.theta = [0.0, 0.0]
        self.theta_0 = [0.0, 0.0]
        self.homed = False
        self.theta_dot = [0.0, 0.0]
        self.tau = 0.0
        self.m1 = m1
        self.m2 = m2
        self.lc1 = lc1
        self.b1 = b1
        self.b2 = b2
        self.l1 = l1
        self.I1 = I1
        self.I2 = I2
        self.gamma_1 = gamma_1
        self.gamma_2 = gamma_2
        self.kp = kp
        self.kv = kv
        self.cmd = 0.0
        self.swing_mode = True
        self.catch_mode = False

    def update_states(self, jstate):
        pos = jstate.position
        vel = jstate.velocity
        self.theta[0] = pos[0]
        self.theta[1] = pos[1]
        self.theta_dot[0] = vel[0]
        self.theta_dot[1] = vel[1]
        # if self.theta_0 == [0.0, 0.0] and not self.homed:
        #     self.theta_0 = [pos[0], pos[1]]
        #     self.homed = True

    def compute_ida_pbc(self):
        q1 = self.theta[0] - pi
        q2 = self.theta[1]
        q1dot = self.theta_dot[0]
        q2dot = self.theta_dot[1]
        m3 = self.m1 + self.m2
        gamma_1 = self.gamma_1
        gamma_2 = self.gamma_2
        kp = self.kp
        kv = self.kv
        qbar = q2 + gamma_2*q1
        qbardot = q2dot + gamma_2*q1dot
        u_es = gamma_1*sin(q1) + kp*sin(qbar)
        u_di = -kv*qbardot

        return u_es + u_di

    def compute_lqr(self):
        q1 = self.theta[0] #- self.theta_0[0] 
        q2 = self.theta[1] #- self.theta_0[1]
        q1dot = self.theta_dot[0]
        q2dot = self.theta_dot[1]
        # if abs(rem2pi(q1-pi)) >= pi/4 or abs(q1dot) >= 10:
        #     self.catch_mode = False
        #     return 0.0
        # K = np.array([-17.22606199290638, -0.31622776601668434, -2.74115091750994, -0.15284235573642457])
        # K = np.array([-5.0395015784599755, -0.010000000000012308, -0.8019036639595001, -0.012460996463504348])
        K = np.array([-7.409595362575457, -0.05000000000000429, -1.1791663255097424, -0.03665716263249201])
        x = np.array([sin(q1-pi), sin(q2), q1dot, q2dot])
        return -np.dot(K,x)

    def switching_control(self):
        q1 = self.theta[0]
        # q2 = self.theta[1]
        q1dot = self.theta_dot[0]
        # q2dot = self.theta_dot[1]
        if (1+cos(q1)) < (1+cos(pi-pi/10)) and abs(q1dot) < 5.0:
            self.catch_mode = True
            return self.compute_lqr()
        else:
            self.catch_mode = False
            return clip(self.compute_ida_pbc(), -0.21, 0.21)

def main():

    ## Initialize ROS node
    rospy.init_node('iwp_swingup_controller', anonymous=True)

    ## Create Acrobot instance
    robot = InertiaWheelPendulum()

    ## Read ROS params
    state_topic_name = rospy.get_param("~state_topic", "acrobot_state")
    effort_topic_name = rospy.get_param("~effort_topic", "acrobot_command")
    deadband = rospy.get_param("~deadband", 0.0)
    max_torque = rospy.get_param("~saturation", 2.5)
    publish_rate = rospy.get_param("~publish_rate", 200)
    rospy.loginfo("[swingup_controller] Publishing torque commands to: %s",     effort_topic_name)
    rospy.loginfo("[swingup_controller] Deadband:                      %f N-m", deadband)
    rospy.loginfo("[swingup_controller] Saturation:                    %f N-m", max_torque)
    rospy.loginfo("[swingup_controller] Publish rate:                  %d Hz",  publish_rate)

    ## Setup pub/sub
    pub = rospy.Publisher(effort_topic_name, Float64, queue_size=1)
    jstate_sub = rospy.Subscriber(state_topic_name, JointState, robot.update_states)
    rate = rospy.Rate(publish_rate)
    def hack_safe_shutdown():
        pub.publish(0.0)
    rospy.on_shutdown(hack_safe_shutdown)

    while not rospy.is_shutdown():
        
        torque = 0.0
        # torque = robot.compute_lqr()
        # torque = robot.compute_ida_pbc()
        torque = robot.switching_control()

        if torque < 0.0:
            cmd = clip(torque, -max_torque, -deadband) 
        elif torque > 0.0:
            cmd = clip(torque, deadband, max_torque)
        else:
            cmd = 0.0
        gear_ratio = 1.0
        eta = 0.95 # 0.88
        k_tau = 0.230    # N-m/a
        current = cmd / gear_ratio / k_tau / eta
        robot.cmd = current
        pub.publish(current)
        
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    

