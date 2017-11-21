import json
import apiai
import string
import random

SID_VALID = string.ascii_uppercase + string.digits


def get_session_id():
	return ''.join(random.choice(SID_VALID) for _ in range(36))
	


class AI:
	
	def __init__(self,cred_file='/home/volant360/catkin_ws/src/inmoov_bringup/tools/routine/credentials/apiai.json'):

		with open(cred_file,'r') as file:
			credentials = json.loads(file.read())
			self.client = apiai.ApiAI(credentials["client_access"])
			self.session_id = get_session_id()
			
	def handle_message(self,message):
		request = self.client.text_request()
		request.session_id = self.session_id
		request.query = message
		response = request.getresponse().read()
		return json.loads(response.decode('utf-8'))