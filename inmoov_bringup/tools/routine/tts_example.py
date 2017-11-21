import json
import pyaudio

from os.path import join, dirname
from watson_developer_cloud import TextToSpeechV1

chunk = 1024
FORMAT = pyaudio.paInt16
CHANNELS = 1
RATE = 16000
RECORD_SECONDS = 5

p = pyaudio.PyAudio()

stream = p.open(
	format = FORMAT,
    channels = CHANNELS,
    rate = RATE,
    output = True,
    frames_per_buffer = chunk
)


text_to_speech = TextToSpeechV1(
    username='56680910-9771-4c1d-a22d-e877bc354fe2',
    password='JOM0XYHFUo7h',
    x_watson_learning_opt_out=True)  # Optional flag




#with open('output.wav','wb') as audio_file:
#    audio_file.write(
#        text_to_speech.synthesize('Hello world! I am Leonardo GreenMoov', accept='audio/wav',
#                                  voice="en-US_MichaelVoice"))
#
#print(
#    json.dumps(text_to_speech.pronunciation(
#        'Watson', pronunciation_format='spr'), indent=2))
#
#print(json.dumps(text_to_speech.customizations(), indent=2))


audio = text_to_speech.synthesize(
	'Hello world! I am Leonardo GreenMoov',
	accept='audio/l16;rate=16000',
	voice='en-US_MichaelVoice'
)

for i in range(0, len(audio), chunk):
    stream.write(audio[i:i+chunk])