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
from inmoov_msgs.msg import PerceptionFrame
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

MAX_ANGLE_CHANGE = 50;
DISABLE_LIMITS = False;

X = 1
Y = 0
Z = 2

class PerceptionListener(object):

    def __init__(self):
        super(PerceptionListener, self).__init__()
        rospy.init_node('perception_listener', anonymous=True)
        self.servos = load_config_from_param()
        self.listado = list()
        self.servicebus = {}
        self.commandbus = {}
        self.jointcommand = JointState()
        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        self.statusSubscriber = rospy.Subscriber("perception_neuron/data", PerceptionFrame, self.data)

        # iterate through servo collection
        for j,b in rospy.get_param('/joints').items():

            number = rospy.get_param('/joints/' + j + '/bus')
            commandbusname = '/servobus/' + str(number).zfill(2) + '/motorcommand'
            servicebusname = '/servobus/' + str(number).zfill(2) + '/motorparameter'

            # index by busnumber, add to collection if busnumber not found in collection
            if not self.servicebus.has_key(number):
                self.commandbus[number] = rospy.Publisher(commandbusname, MotorCommand, queue_size=10)
                self.servicebus[number] = rospy.ServiceProxy(servicebusname, MotorParameter)
        rospy.spin()

    def data(self, data):
        self.moveTo("l_index_joint", data.leftHandIndex)
        self.moveTo("l_thumb_joint", data.leftHandThumb)
        self.moveTo("l_middle_joint",  data.leftHandMiddle)
        self.moveTo("l_ring_joint", data.leftHandRing)
        self.moveTo("l_pinky_joint", data.leftHandPinky)

        self.moveTo("r_index_joint", data.rightHandIndex)
        self.moveTo("r_thumb_joint", data.rightHandThumb)
        self.moveTo("r_middle_joint", data.rightHandMiddle)
        self.moveTo("r_ring_joint", data.rightHandRing)
        self.moveTo("r_pinky_joint", data.rightHandPinky)

        self.moveTo("l_shoulder_out_joint", data.leftOmoplate)
	self.moveTo("r_shoulder_out_joint", data.rightOmoplate)

	self.moveTo("l_elbow_flex_joint", data.leftBicep)
	self.moveTo("r_elbow_flex_joint", data.rightBicep)
	
	self.moveTo("l_shoulder_lift_joint", data.leftShoulder)
	self.moveTo("r_shoulder_lift_joint", data.rightShoulder)


    # validates if the change is less than the secure angel established at
    # MAX_ANGLE_CHANGE
    def validate(self, actual_val, new_val):
        # print ">>>>> " + str(abs(actual_val - new_val))
        return abs(actual_val - new_val) < MAX_ANGLE_CHANGE;
        # return True;

    def moveTo(self, name, value):
        s = self.servos[name]

        if(self.validate(float(s.goal), value) or DISABLE_LIMITS):
            if(value > float(s.minGoal) and value < float(s.maxGoal) or DISABLE_LIMITS):
                s.goal = value

                # send to arduino directly
                motorcommand = MotorCommand()
                motorcommand.id = int(s.servoPin)
                motorcommand.parameter = PROTOCOL.GOAL

                if bool(s.inverted):
                    motorcommand.value = float(s.maxGoal) - float(value) + float(s.minGoal)
                #print "inversed " + str(value) + "->" + str(motorcommand.value)
                else:
                    motorcommand.value = value
                # print "[" +  str(s.bus) + "]->" + str(motorcommand.value)
                # motorcommand.value = 10.0
                self.commandbus[s.bus].publish(motorcommand)

                # print "Moving " + name + " to " + str(motorcommand.value)

                # send to joint_command
                self.jointcommand.name = []
                self.jointcommand.position = []

                self.jointcommand.header = Header()
                self.jointcommand.header.stamp = rospy.Time.now()
                self.jointcommand.name.append(name)
                # if parameter == PROTOCOL.GOAL:
                # if bool(s.inverted):
                #     self.jointcommand.position.append(float(s.maxGoal) - float(value) + float(s.minGoal))
                #     # print "inversed " + str(value) + "->" + str(motorcommand.value)
                # else:
                #     self.jointcommand.position.append(value)
                # else:
                #     self.jointcommand.position.append(value)
                self.jointcommand.position.append(float(value))
                self.jointcommand.velocity = []
                self.jointcommand.effort= []
                self.jointPublisher.publish(self.jointcommand)
        else:
            print "No se puede mover " + name + str(value)


    def rad2Grad(self, rad):
        return rad*180/3.1416;

def main():
	perception_listener = PerceptionListener()

if __name__ == '__main__':  # if we're running file directly and not importing it
	main()

