#!/usr/bin/python

import sys
import os
import rospy
import rospkg

import yaml

from inmoov_msgs.msg import MotorStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from std_msgs.msg import Float64MultiArray
from time import sleep

import os
import sys
from os.path import dirname, abspath


#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))),'include'))

from constants import PROTOCOL
from servos import Servo

from load_config_from_param import load_config_from_param

class HeadTracking(object):

    def __init__(self):
        super(HeadTracking, self).__init__()
        rospy.init_node('head_tracking', anonymous=True)

        self.servos = load_config_from_param()
        self.servicebus = {}
        self.commandbus = {}
        self.jointcommand = JointState()
        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        self.statusSubscriber = rospy.Subscriber("oculus/data", Float64MultiArray, self.head_track)
        rospy.spin()
        # iterate through servo collection
        for j,b in rospy.get_param('/joints').items():

            number = rospy.get_param('/joints/' + j + '/bus')
            commandbusname = '/servobus/' + str(number).zfill(2) + '/motorcommand'
            servicebusname = '/servobus/' + str(number).zfill(2) + '/motorparameter'

            # index by busnumber, add to collection if busnumber not found in collection
            if not self.servicebus.has_key(number):
                self.commandbus[number] = rospy.Publisher(commandbusname, MotorCommand, queue_size=10)
                self.servicebus[number] = rospy.ServiceProxy(servicebusname, MotorParameter)

    def head_track(self, data):
        yaw 	= self.rad2Grad(data.data[0])
        pitch 	= self.rad2Grad(data.data[1])
        roll	= self.rad2Grad(data.data[2])
        self.moveTo("head_pan_joint" , yaw)
        self.moveTo("head_tilt_joint" , pitch)
        print "Yaw: " + str(yaw) + " pitch: " + str(pitch) + " roll: " + str(roll)

    def moveTo(self, name, value):
        s = self.servos[name]

        # send to arduino directly
        motorcommand = MotorCommand()
        motorcommand.id = int(s.servoPin)
        motorcommand.parameter = PROTOCOL.GOAL

        if bool(s.inverted):
            motorcommand.value = float(s.maxGoal) - float(value)
        #print "inversed " + str(value) + "->" + str(motorcommand.value)
        else:
            motorcommand.value = value
        # print "[" +  str(s.bus) + "]->" + str(motorcommand.value)
        # motorcommand.value = 10.0
        ##self.commandbus[s.bus].publish(motorcommand)

        print "Moving " + name + " to " + str(motorcommand.value)

        # send to joint_command
        self.jointcommand.name = []
        self.jointcommand.position = []

        self.jointcommand.header = Header()
        self.jointcommand.header.stamp = rospy.Time.now()
        self.jointcommand.name.append(name)
        # if parameter == PROTOCOL.GOAL:
        #     if bool(s.inverted):
        #         self.jointcommand.position.append(float(s.maxGoal) - float(value))
        #         # print "inversed " + str(value) + "->" + str(motorcommand.value)
        #     else:
        #         self.jointcommand.position.append(value)
        # else:
        #     self.jointcommand.position.append(value)
        self.jointcommand.position.append(value)
        self.jointcommand.velocity = []
        self.jointcommand.effort= []
        self.jointPublisher.publish(self.jointcommand)

    def rad2Grad(self, rad):
        return rad*180/3.1416;

def main():
	head_traking = HeadTracking()

if __name__ == '__main__':  # if we're running file directly and not importing it
	main()
