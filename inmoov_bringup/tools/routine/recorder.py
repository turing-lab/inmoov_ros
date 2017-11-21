import pyaudio
import threading

class Recorder(threading.Thread):


    ##Para que funcione, los frames_per_buffer del stream
    ##deben ser iguales a la variable frames_per_buffer
    def __init__(self,stream,frames_per_buffer=1024):
        threading.Thread.__init__(self)
        self.condition = threading.Condition()
        self.stream = stream
        self.chunk_size = frames_per_buffer
        self.data = None
        self.data_ready = False
        self.running = False


    def run(self):
        while(True):
            print('STARTING LOOP')
            self.condition.acquire()
            self.condition.wait()

            self.data_ready = False
            self.running = True
            self.stream.start_stream()
            frames = []

            while self.running:
                data = self.stream.read(self.chunk_size)
                frames.append(data)

            self.stream.stop_stream()
            self.data = b''.join(frames)
            self.data_ready = True

    def stop(self):
        self.running = False
