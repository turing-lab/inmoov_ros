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
DISABLE_LIMITS = True;

X = 0
Y = 1
Z = 2

# Body
HIPS_POSITION 		= 0;
HIPS 				= 1;
RIGHT_UP_LEG 		= 2;
RIGHT_LEG 			= 3;
RIGHT_FOOT 			= 4;
LEFT_UP_LEG 		= 5;
LEFT_LEG 			= 6;
LEFT_FOOT 			= 7;
SPINE 				= 8;
SPINE1				= 9;
SPINE2				= 10;
SPINE3				= 11;
NECK				= 12;
HEAD				= 13;
RIGHT_SHOULDER 		= 14;
RIGHT_ARM 			= 15;
RIGHT_FORE_ARM 		= 16;
RIGHT_HAND			= 17;

# Right Fingers
RIGHT_HAND_THUMB1	= 18;
RIGHT_HAND_THUMB2	= 19;
RIGHT_HAND_THUMB3	= 20;

RIGHT_IN_HAND_INDEX	= 21;
RIGHT_HAND_INDEX1	= 22;
RIGHT_HAND_INDEX2	= 23;
RIGHT_HAND_INDEX3	= 24;
RIGHT_IN_HAND_MIDDLE	= 25;
RIGHT_HAND_MIDDLE1	= 26;
RIGHT_HAND_MIDDLE2	= 27;
RIGHT_HAND_MIDDLE3	= 28;
RIGHT_IN_HAND_RING	= 29;
RIGHT_HAND_RING1	= 30;
RIGHT_HAND_RING2	= 31;
RIGHT_HAND_RING3	= 32;
RIGHT_IN_HAND_PINKY	= 33;
RIGHT_HAND_PINKY1	= 34;
RIGHT_HAND_PINKY2	= 35;
RIGHT_HAND_PINKY3	= 36;

# Body
LEFT_SHOULDER 		= 37;
LEFT_ARM 			= 38;
LEFT_FORE_ARM 		= 39;
LEFT_HAND			= 40;

######
# data 2
######

# Left Fingers
LEFT_HAND_THUMB1	= 41;
LEFT_HAND_THUMB2	= 42;
LEFT_HAND_THUMB3	= 43;
LEFT_IN_HAND_INDEX	= 44;
LEFT_HAND_INDEX1	= 45;
LEFT_HAND_INDEX2	= 46;
LEFT_HAND_INDEX3	= 47;
LEFT_IN_HAND_MIDDLE	= 48;
LEFT_HAND_MIDDLE1	= 49;
LEFT_HAND_MIDDLE2	= 50;
LEFT_HAND_MIDDLE3	= 51;
LEFT_IN_HAND_RING	= 52;
LEFT_HAND_RING1		= 53;
LEFT_HAND_RING2		= 54;
LEFT_HAND_RING3		= 55;
LEFT_IN_HAND_PINKY	= 56;
LEFT_HAND_PINKY1	= 57;
LEFT_HAND_PINKY2	= 58;
LEFT_HAND_PINKY3	= 59;

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
        self.statusSubscriber = rospy.Subscriber("perception_neuron/data_1", Float64MultiArray, self.data_1)
        self.statusSubscriber = rospy.Subscriber("perception_neuron/data_2", Float64MultiArray, self.data_2)
        self.statusSubscriber = rospy.Subscriber("perception_neuron/data_3", Float64MultiArray, self.data_3)

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
        self.moveTo("l_index_joint", 0 - data.leftHandIndex)
        self.moveTo("l_thumb_joint", data.leftHandThumb)
        self.moveTo("l_middle_joint", 0 - data.leftHandMiddle)
        self.moveTo("l_ring_joint", 0 - data.leftHandRing)
        self.moveTo("l_pinky_joint", 0 - data.leftHandPinky)

        self.moveTo("r_index_joint", 0 - data.rightHandIndex)
        self.moveTo("r_thumb_joint", data.rightHandThumb)
        self.moveTo("r_middle_joint", 0 - data.rightHandMiddle)
        self.moveTo("r_ring_joint", 0 - data.rightHandRing)
        self.moveTo("r_pinky_joint", 0 - data.rightHandPinky)

        self.moveTo("l_shoulder_out_joint", data.leftOmoplate)


    def data_1(self, data):
        # print "data_1 " + str(data.layout.data_offset)
        if(True):
            self.moveTo("r_index_joint", 0 - data.data[RIGHT_HAND_INDEX1*3 + Z])
            self.moveTo("r_thumb_joint", data.data[RIGHT_HAND_THUMB2*3 + Z ])
            self.moveTo("r_middle_joint", 0 - data.data[RIGHT_HAND_MIDDLE1*3 + Z])
            self.moveTo("r_ring_joint", 0 - data.data[RIGHT_HAND_RING1*3 + Z ])
            self.moveTo("r_pinky_joint", 0 - data.data[RIGHT_HAND_PINKY1*3 + Z])

        self.moveTo("l_elbow_flex_joint", 10 - data.data[LEFT_FORE_ARM*3 + Z])
        self.listado.append(data.data[RIGHT_FORE_ARM*3 + Z])
        print "min: " + str(min(self.listado)) + " max: " + str(max(self.listado))
        self.moveTo("r_elbow_flex_joint", data.data[RIGHT_FORE_ARM*3 + Z] - 10)

        # # self.moveTo("l_shoulder_out_joint", 120 - data.data[LEFT_ARM*3 + Y])

        self.moveTo("l_upper_arm_roll_joint", 120 - data.data[LEFT_ARM*3 + X])

        self.moveTo("r_upper_arm_roll_joint", 50.81 - data.data[RIGHT_ARM*3 + X])

        # yaw 	= self.rad2Grad(data.data[0])
        # pitch 	= self.rad2Grad(data.data[1])
        # roll	= self.rad2Grad(data.data[2])
        # self.moveTo("head_pan_joint" , yaw)
        # self.moveTo("head_tilt_joint" , pitch)
        # print "Yaw: " + str(yaw) + " pitch: " + str(pitch) + " roll: " + str(roll)

    def data_2(self, data):
        self.moveTo("l_thumb_joint", data.data[LEFT_HAND_THUMB2*3 + Z - 120])
        self.moveTo("l_index_joint", 0 - data.data[LEFT_HAND_INDEX1*3 + Z - 120])
        self.moveTo("l_middle_joint", 0 - data.data[LEFT_HAND_MIDDLE1*3 + Z - 120])
        self.moveTo("l_ring_joint", 0 - data.data[LEFT_HAND_RING1*3 + Z - 120])
        self.moveTo("l_pinky_joint", 0 - data.data[LEFT_HAND_PINKY1*3 + Z - 120])

        print "data_2 " + str(data.layout.data_offset)

    def data_3(self, data):
        print "data_3 " + str(data.layout.data_offset)

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

                print "Moving " + name + " to " + str(motorcommand.value)

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
