import pyaudio
import json
import mover
import time
from watson_developer_cloud import TextToSpeechV1

def hello(self):
	self.moveTo("l_elbow_flex_joint", 90)
	time.sleep(0.2)
	self.moveTo("l_wrist_roll_joint", 135)
	time.sleep(4.6)
	self.moveTo("l_thumb_joint", 180)
	self.moveTo("l_index_joint", 0)
	self.moveTo("l_middle_joint", 0)
	self.moveTo("l_ring_joint", 180)
	self.moveTo("l_pinky_joint", 180)
	time.sleep(1.6)
	self.moveTo("l_thumb_joint", 0)
	self.moveTo("l_index_joint", 180)
	self.moveTo("l_middle_joint", 180)
	self.moveTo("l_ring_joint", 0)
	self.moveTo("l_pinky_joint", 0)
	time.sleep(0.5)
	self.moveTo("l_elbow_flex_joint", 40)
	

def take_tool(self):
	self.moveTo("l_elbow_flex_joint", 90)
	for i in range (30,80):
	    self.moveTo("l_shoulder_lift_joint",i)
	    #self.moveTo("r_shoulder_lift_joint",i-30)
	    self.moveTo("l_shoulder_out_joint", (80+(i*0.3)))
	    #self.moveTo("r_shoulder_out_joint", (90+(i*0.3)))
	    time.sleep(0.05)
	time.sleep(2)   
	#cerramos mano derecha         
	#cerramos mano izquierda
	self.moveTo("l_index_joint",0)
	self.moveTo("l_thumb_joint",180)
	self.moveTo("l_middle_joint",0)
	self.moveTo("l_pinky_joint",180)
	self.moveTo("l_ring_joint",180)

	for i in range (80,30,-1):
	    self.moveTo("l_shoulder_lift_joint",i)
	    self.moveTo("l_shoulder_out_joint", (80+(i*0.3))) 
	    time.sleep(0.05)   

	time.sleep(1) 
	self.moveTo("l_shoulder_out_joint",80)
	self.moveTo("r_shoulder_out_joint",90) 
	#abrir

def give_tool(self):
	self.moveTo("l_elbow_flex_joint", 90)
	for i in range (30,80):
	    self.moveTo("l_shoulder_lift_joint",i)
	    self.moveTo("r_shoulder_lift_joint",i-30)
	    self.moveTo("l_shoulder_out_joint", (80+(i*0.3)))
	    self.moveTo("r_shoulder_out_joint", (90+(i*0.3)))
	    sleep(0.05)
	sleep(2)   

	#cerramos mano izquierda
	for i in range (80,30,-1):
	    self.moveTo("l_shoulder_lift_joint",i)
	    self.moveTo("r_shoulder_lift_joint",i-30)
	    self.moveTo("l_shoulder_out_joint", (80+(i*0.3)))
	    self.moveTo("r_shoulder_out_joint", (90+(i*0.3)))
	    sleep(0.05)
	   

	sleep(1) 
	self.moveTo("l_shoulder_out_joint",80)
	self.moveTo("r_shoulder_out_joint",90) 
	#abrir
	sleep(6)
	# mano izquierda

	sleep(5)




##THIS CLASS ALSO TAKES CARE OF LEONARDO MOVEMENT
class TTS:

	def __init__(self,stream,cred_file='/home/volant360/catkin_ws/src/inmoov_bringup/tools/routine/credentials/ibm_watson.json'):
		with open(cred_file,'r') as file:
			credentials = json.loads(file.read())
			self.client = TextToSpeechV1(
	    		username=credentials['username'],
	    		password=credentials['password'],
	    		x_watson_learning_opt_out=True
	    		)
		self.stream = stream
		self.leo = mover.Mover()
		self.leo.enableServosL()
		self.leo.enableServosR()
		self.leo.start()
		self.leo_mouth = mover.Mover(mouth=True)
		self.leo_mouth.enable("head_pan_joint")
		self.leo_mouth.enable("head_tilt_joint")
		self.leo_mouth.enable("jaw_joint")
		self.leo_mouth.start()


	def handle_action(self,message):
		if message.find('Hello ') != -1:
			self.leo.set_move_function(hello)
			self.leo.condition.acquire()
			self.leo.condition.notify()
			self.leo.condition.release()
		elif message.find(' this tool ') != -1:
			self.leo.set_move_function(take_tool)
			self.leo.condition.acquire()
			self.leo.condition.notify()
			self.leo.condition.release()


	def activate_mouth(self):
		self.leo_mouth.condition.acquire()
		self.leo_mouth.condition.notify()
		self.leo_mouth.condition.release()

	def stop_mouth(self):
		self.leo_mouth.speak = False

	##audio rate and chunk size must fit that of stream
	def talk(self,message,accept='audio/l16;rate=16000',chunk_size=1024):
		self.stream.start_stream()

		audio = self.client.synthesize(
			message,
			accept=accept,
			voice='en-US_MichaelVoice'
			)

		self.handle_action(message)
		self.activate_mouth()
		for i in range(0, len(audio), chunk_size):
			self.stream.write(audio[i:i+chunk_size],exception_on_underflow=False)
		self.stop_mouth()
		self.stream.stop_stream()



