#!/usr/bin/env python

import recorder
import stt
import tts
import ai
import pyaudio
import rospy
import rospkg
import mover

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

def main():
    audio = pyaudio.PyAudio()
    in_stream = audio.open(format=pyaudio.paInt16,channels=1,
            rate=16000,input=True,frames_per_buffer=1024)
    out_stream = audio.open(format=pyaudio.paInt16,channels=1,
            rate=16000,output=True,frames_per_buffer=1024)
    rec = recorder.Recorder(in_stream)
    stt_client = stt.STT()
    ai_client = ai.AI()
    tts_client = tts.TTS(out_stream)
    rec.start()
    print('%s SETUP READY %s'%('-'*38,'-'*38))

    copyOn = False

    for i in range(20):
        sys.stdin.readline()
        rec.condition.acquire()
        rec.condition.notify()
        rec.condition.release()
        print('%s REC   START %s'%('-'*38,'-'*38))
        sys.stdin.readline()
        print('%s REC    STOP %s'%('-'*38,'-'*38))
        rec.stop()
        
        while (not rec.data_ready):
            continue

        stt_response = stt_client.transcribe(rec.data)
        try:
            entry = stt_response.results[0].alternatives[0]
            message =  entry.transcript
            acc = entry.confidence
        except IndexError:
            message = ''
            acc = 0.0


        print('%s TRANSCRIPT:%s %s'%('-'*20,message,'-'*20))
        response = ''

        if copyOn:      
            if message.strip().lower() == 'finish':
                copyOn = False
                response = 'Ok! I will exit copy mode now.'
            else:
                response = message
        else:
            result = ai_client.handle_message(message)
        
            contexts = result['result']['contexts']
            if len(contexts) > 0:
                if contexts[0]['name'] == 'copy':
                    print('entering copy mode')
                    copyOn = True
            response = result['result']['fulfillment']['speech']

        print('%s LOOP FINISHED %s'%('-'*38,'-'*38))
        tts_client.talk(response)





    in_stream.close()
    out_stream.close()
    audio.terminate()
    print('DONE!')
    sys.exit(0)



if __name__ == '__main__':  # if we're running file directly and not importing it
    main()
