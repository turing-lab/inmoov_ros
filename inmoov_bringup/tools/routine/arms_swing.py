#!/usr/bin/env python

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

from time import sleep

import os
import sys
from os.path import dirname, abspath


#hacky way to add include directory to sys path
sys.path.append(os.path.join(dirname(dirname(dirname(abspath(__file__)))),'include'))

from constants import PROTOCOL
from servos import Servo

from load_config_from_param import load_config_from_param
class Routine(object):

    def __init__(self):
        super(Routine, self).__init__()
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
            if not self.servicebus.has_key(number):
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
        print "Enable " + servo

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
        self.enable("r_shoulder_lift_joint") #shouldet
        self.enable("r_shoulder_out_joint") #omoplate

    def enableServosH(self): 
        self.enable("head_pan_joint")#JAW
        self.enable("head_tilt_joint")#PITCH
        self.enable("jaw_joint") 


    def definitions(self):
        # while False:
        while not rospy.is_shutdown():
            
            for i in range (60,40,-1):
                self.moveTo("head_tilt_joint",i)
                sleep(0.04)

            for i in range (100,40,-1):
                self.moveTo("l_elbow_flex_joint",i)
                self.moveTo("r_elbow_flex_joint",140-i)
                self.moveTo("l_shoulder_lift_joint",30+(i*0.4))
                self.moveTo("r_shoulder_lift_joint",60-(i*0.4))
                self.moveTo("l_shoulder_out_joint",65+(0.4*i))
                self.moveTo("r_shoulder_out_joint",130-(0.4*i))
                self.moveTo("head_pan_joint",(i+25))
                sleep(0.05)

            sleep(1)
                
            for i in range (40,100):
                self.moveTo("l_elbow_flex_joint",i)
                self.moveTo("r_elbow_flex_joint",140-i)
                self.moveTo("l_shoulder_lift_joint",30+(i*0.4))
                self.moveTo("r_shoulder_lift_joint",60-(i*0.4))
                self.moveTo("l_shoulder_out_joint",65+(0.4*i))
                self.moveTo("r_shoulder_out_joint",130-(0.4*i))
                self.moveTo("head_pan_joint",(i+25))
                sleep(0.05)

            for i in range (40,60):
                self.moveTo("head_tilt_joint",i)
                sleep(0.04)
            sleep(1)    

def main():
    routine = Routine()
    sleep(1)
    routine.enableServosL()
    routine.enableServosR()
    routine.enableServosH()
    routine.definitions()



if __name__ == '__main__':  # if we're running file directly and not importing it
    main() 