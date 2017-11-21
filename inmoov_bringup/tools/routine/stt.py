from google.cloud import speech
from google.cloud.speech import enums
from google.cloud.speech import types
import pyaudio
import wave
import io

class STT:

    def __init__(self):
        self.client = speech.SpeechClient()

    def transcribe(self,content):

        audio = types.RecognitionAudio(content=content)
        config = types.RecognitionConfig(
                encoding=enums.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=16000,
                language_code='en-US')

        return self.client.recognize(config, audio)