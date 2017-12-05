#!/usr/bin/env python

import sys
import os
import rospy
import rospkg
import yaml
import json
import threading

from inmoov_msgs.msg import MotorStatus
from inmoov_msgs.msg import MotorCommand
from inmoov_msgs.srv import MotorParameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from time import sleep
from os.path import dirname, abspath


#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))),'include'))

from constants import PROTOCOL
from servos import Servo

from load_config_from_param import load_config_from_param
class Mover(threading.Thread):

    def __init__(self):
        threading.Thread.__init__(self)
        rospy.init_node('routine', anonymous=True)

        self.servos = load_config_from_param()
        self.servicebus = {}
        self.commandbus = {}
        self.jointcommand = JointState()
        self.jointPublisher = rospy.Publisher("joint_command", JointState, queue_size=10)
        # iterate through servo collection
        for j,b in rospy.get_param('/joints').items():

            number = rospy.get_param('/joints/' + j + '/bus')
            commandbusname = '/servobus/' + str(number).zfill(2) + '/motorcommand'
            servicebusname = '/servobus/' + str(number).zfill(2) + '/motorparameter'

            # index by busnumber, add to collection if busnumber not found in collection
            if number not in self.servicebus:
                self.commandbus[number] = rospy.Publisher(commandbusname, MotorCommand, queue_size=10)
                self.servicebus[number] = rospy.ServiceProxy(servicebusname, MotorParameter)

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
        self.commandbus[s.bus].publish(motorcommand)

        print("Moving %s to %f"%(name,motorcommand.value))

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

    def moveHand(self, side, value):
        self.moveTo(side + "_index_joint", value)
        self.moveTo(side + "_thumb_joint", value)
        self.moveTo(side + "_middle_joint", value)
        self.moveTo(side + "_pinky_joint", value)
        self.moveTo(side + "_ring_joint", value)

    def enable(self, servo):
        s = self.servos[servo]

        # send to arduino directly
        motorcommand = MotorCommand()
        motorcommand.id = int(s.servoPin)
        motorcommand.parameter = PROTOCOL.ENABLE
        motorcommand.value = float(1.0)
        self.commandbus[s.bus].publish(motorcommand)
        print("Enable %s"%(servo))

        return True;

    def enableServosL(self):

        self.enable("l_thumb_joint")
        self.enable("l_index_joint")
        self.enable("l_middle_joint")
        self.enable("l_pinky_joint")
        self.enable("l_ring_joint")

        self.enable("l_elbow_flex_joint") #bicep
        self.enable("l_upper_arm_roll_joint") # rotate
        self.enable("l_shoulder_lift_joint") #shoulder
        self.enable("l_shoulder_out_joint") #omoplate

    def enableServosR(self):
        self.enable("r_thumb_joint")
        self.enable("r_index_joint")
        self.enable("r_middle_joint")
        self.enable("r_pinky_joint")
        self.enable("r_ring_joint")

        self.enable("r_elbow_flex_joint") #bicep
        self.enable("r_upper_arm_roll_joint") # rotate
        self.enable("r_shoulder_lift_joint") #shoulder
        self.enable("r_shoulder_out_joint") #omoplate

    ##this is a dummy function
    def move_function(self):
        return

    def set_move_function(self,move_function):
        self.move_function = move_function

    def start():
        while(True):
            self.condition.acquire()
            self.condition.wait()
            self.move_function()

def handle_action(text,routine):
    value = text.find(' hello ')
    if value != -1:
        routine.definitions()


def process_event(event,routine):
    if event.type == EventType.ON_RECOGNIZING_SPEECH_FINISHED:
        handle_action(event.args['text'],routine)
    print(event)