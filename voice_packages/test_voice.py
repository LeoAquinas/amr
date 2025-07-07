#!/home/jetson/venvs/kokoro/bin/python3

import speech_recognition as sr
import ollama
from faster_whisper import WhisperModel
import tempfile
import os
import threading
import queue
import time
import noisereduce as nr
import numpy as np
import soundfile as sf
import subprocess

from kokoro_onnx import Kokoro
import sounddevice as sd
import re

import onnxruntime
from onnxruntime import InferenceSession
# TODO: remove as not being used
from select import select
import sys

# ROS2       
import rclpy
from std_msgs.msg import String

# Use GPU if available (add device="cuda" if you have NVIDIA GPU)
model = WhisperModel("tiny.en", device="cpu", compute_type="int8")
response_queue = queue.Queue()  # For streaming chunks
kokoro_queue = queue.Queue()

interrupt_event = threading.Event() # Interrupt thread

# Initialize Kokoro ONNX
ONNX_PROVIDER = "CUDAExecutionProvider"  # "CPUExecutionProvider"
session = InferenceSession("/home/jetson/agv/src/amr/kokoro/kokoro-v1.0.onnx", providers=[ONNX_PROVIDER])
kokoro = Kokoro.from_session(session, "/home/jetson/agv/src/amr/kokoro/voices-v1.0.bin")

# kokoro = Kokoro(
#     model_path="/home/jetson/agv/src/kokoro-v1.0.onnx",
#     voices_path="/home/jetson/agv/src/voices-v1.0.bin"
# )

_cleanup = re.compile(r'[\*]')

sd.default.blocksize = 2048
sd.default.latency = 'low'

# List of words were obtained from the Cambrdidge Thesaurus of english
# To move forwards
forward_command_list = ['forward', 'front']
# To move backwards
backward_command_list = ['reverse', 'back', 'pull back', 'back up', 'backwards', 'backward',]
# To move left
left_command_list = ['left']
# To move right
right_command_list = ['right']
# To stop
stop_command_list = ['stop']

nav_list = ['mvi lab', 'machine lab', 'vision lab', 'program lab', 'robotics lab', 'robotic lab', 'design lab', 'machine vision lab']

# TODO:
# Modify to suit speech input rather than keyboard
def wait_for_interrupt():
    """Monitor stdin for Enter key press"""
    while True:
        # Non-blocking check for Enter press
        rlist, _, _ = select([sys.stdin], [], [], 0.1)
        if rlist:
            if sys.stdin.read(1) == '\n':
                interrupt_event.set()
                break

def play_audio(samples, sample_rate):
    """Play audio directly without saving to file"""
    try:
        sd.play(samples, sample_rate)
        sd.wait()  # Block until playback finishes
    except Exception as e:
        print(f"Audio playback error: {e}", flush=True)

def tts_worker():
    """Handles text-to-speech generation with sentence accumulation"""
    sentence_buffer = ""
    first_input = True  # Track if this is the very first text chunk

    while True:
        text = kokoro_queue.get()

        if text is None or interrupt_event.is_set():
            # Process remaining text in buffer
            if sentence_buffer:
                try:
                    samples, sample_rate = kokoro.create(
                        sentence_buffer,
                        voice="af_sarah",
                        speed=1.0,
                        lang="en-us"
                    )
                    play_audio(samples, sample_rate)
                except Exception as e:
                    print(f"Final TTS Error: {e}", flush=True)
            break

        if text == "<RESET>":
            sentence_buffer = ""
            continue

        try:
            # On the very first incoming text, prepend dummy string
            if first_input:
                dummy = "."  # or just " " or any short delay string
                sentence_buffer += dummy
                first_input = False

            # Add new text to buffer normally
            sentence_buffer += text

            # Check for sentence boundaries
            while True:
                punctuations = {'.', '!', '?', '！', '？'}
                found = None
                for i, char in enumerate(sentence_buffer):
                    if char in punctuations:
                        found = i
                        break

                if found is None:
                    break  # No complete sentence yet

                sentence = sentence_buffer[:found+1].strip()
                sentence_buffer = sentence_buffer[found+1:].lstrip()

                samples, sample_rate = kokoro.create(
                    sentence,
                    voice="af_sarah",
                    speed=1.0,
                    lang="en-us"
                )
                if not interrupt_event.is_set():
                    #global start_speech_response
                    #start_speech_response = time.time()
                    #print(start_speech_response)
                    #print("done")
                    play_audio(samples, sample_rate)

        except Exception as e:
            print(f"TTS Error: {e}", flush=True)

def reduce_noise(audio_data, sample_rate):
    # Convert audio data into numpy array
    audio_array = np.frombuffer(audio_data, dtype=np.int16)
    
    # Perform noise reduction
    reduced_audio = nr.reduce_noise(y=audio_array, sr=sample_rate)
    
    return reduced_audio

def speech_to_text():
    recognizer = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("Adjusting for ambient noise...", flush=True)
        recognizer.adjust_for_ambient_noise(source, duration=3)
        # recognizer.energy_threshold = 500  # Default is ~300, tweak lower or higher based on noise
        print("Listening... (speak now)", flush=True)
        
        try:
            # Capture audio using speech_recognition
            audio = recognizer.listen(source, timeout=7)
            # Convert the audio to numpy array
            audio_data = audio.get_raw_data()

            # Reduce noise
            sample_rate = audio.sample_rate
            reduced_audio = nr.reduce_noise(y=np.frombuffer(audio_data, dtype=np.int16), sr=sample_rate)

            global speech_end_time
            speech_end_time = time.time()
            print(speech_end_time)
            
            # Save audio to temporary file
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                sf.write(fp.name, reduced_audio, sample_rate)
                temp_path = fp.name
            
            # Transcribe with Whisper
            segments, _ = model.transcribe(temp_path)
            text = " ".join([segment.text for segment in segments])
            
            # Clean up temp file
            os.remove(temp_path)

            if text=='':
                return "No speech detected"
            else:
                return text
            
        except sr.WaitTimeoutError:
            return "No speech detected"
        except Exception as e:
            return f"Error: {str(e)}"
        
def sr_quick_command():
    recognizer = sr.Recognizer()
    with sr.Microphone() as source:
        print("Listening for command (PTT)...", flush=True)
        recognizer.adjust_for_ambient_noise(source, duration=1.0)
        recognizer.pause_threshold = 1.5  # Wait a bit longer after speaking ends
        try:
            audio = recognizer.listen(source, timeout=5)
            return recognizer.recognize_google(audio).lower()
        except sr.UnknownValueError:
            return "unrecognized"
        except sr.RequestError as e:
            return f"API error: {e}"
        except sr.WaitTimeoutError:
            return "timeout"

# def response():
def response(input):
    try:
        # Start streaming response
        stream = ollama.generate(
            model='gemma3:1b',
            # prompt='Why is the sky blue?',
            prompt =input,
            #options={'temperature': 0.3, 'num_predict': 200},   # Can consider removing this
            stream=True  # Enable streaming
        )

        # Send chunks to queue
        for chunk in stream:
            if interrupt_event.is_set():    # Check for interruption
                break
            if chunk['response']:
                response_queue.put(chunk['response'])   # Seperate response into chunks and store in queue
                cleaned_text = _cleanup.sub('', chunk['response'])
                kokoro_queue.put(cleaned_text) 
                
        response_queue.put(None)  # Terminate queue to indicate end of stream
        kokoro_queue.put(None) 
        
    except Exception as e:
        response_queue.put(f"\nError: {str(e)}")

# TODO: UNCOMMENT
def start_threads():
    # Clear queues
    while not kokoro_queue.empty():
        kokoro_queue.get()
    kokoro_queue.put("<RESET>")  # Signal to reset sentence buffer

    # Start streaming thread
    threading.Thread(
        target=response,    # Run stream function in background that updates queue
        args=(result,),
        daemon=True         # Kills thread if main program exits
    ).start()

    # Start TTS thread
    threading.Thread(
        target=tts_worker,
        daemon=True
        ).start()

    # Start interrupt check thread
    threading.Thread(
        target=wait_for_interrupt,
        daemon=True
    ).start()


if __name__ == "__main__":

    # Initialize ROS2
    rclpy.init()
    node = rclpy.create_node('voice_control')

    # Create publisher for String messages
    pub = node.create_publisher(String, 'voice_commands', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    trigger = 0
    stop_command = 0
    stop_chat = 0
    nav=0
    while True:
        # TODO: UNCOMMENT
        print("Press Enter to speak...wait for 3 seconds", flush=True)
        if trigger == 0:
            # input("Press Enter to speak...")
            input()
            sd.stop()
            interrupt_event.set()

            while not kokoro_queue.empty():
                try:
                    kokoro_queue.get_nowait()
                except queue.Empty:
                    break

            kokoro_queue.put("<RESET>")
            interrupt_event.clear()
        # Speech to text with ollama
        # if trigger == 1:
        #     result = sr_quick_command()
        # else:
        result = speech_to_text()
        print(f"You said: {result}\n", flush=True)
        publish_msg = 0

        # Retry response if invalid speech input
        if "No speech detected" in result or "Error:" in result:
            continue

        # Clean commands, changing to lower case to compare with list
        cleaned_result = result.strip().lower()
        # cleaned_result = 'forward'

        # Seperate result using regex
        # \w+[\w']* matches:
        # \w+: One or more word characters (letters/digits)
        # [\w']*: Optional continuation of word characters or apostrophes (for contractions like "don't")
        # [,.?] matches individual punctuation marks (,, ., ?)
        separated_cleaned_result = re.findall(r"\w+[\w']*|[,.?]", cleaned_result)
        print(separated_cleaned_result, flush=True)

        # Create String message
        msg = String()

        for command in separated_cleaned_result:
            if command in forward_command_list:
                msg.data = 'forward'
                publish_msg = 1
                stop_chat = 1
            elif command in backward_command_list:
                msg.data = 'backward'
                publish_msg = 1
                stop_chat = 1
            elif command in left_command_list:
                msg.data = 'left'
                publish_msg = 1
                stop_chat = 1
            elif command in right_command_list:
                msg.data = 'right'
                publish_msg = 1
                stop_chat = 1
            elif command in stop_command_list:
                msg.data = 'stop'
                publish_msg = 0
                trigger = 0
                stop_chat = 0
                stop_command = 1

        for points in nav_list:
            if points in cleaned_result:
                if points == 'mvi lab' or points =='machine lab' or points=='vision lab' or points=='machine vision lab':
                    try:
                        # Launch the navigation script
                        subprocess.Popen(["python3", "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/mvi.py"])
                        print("Navigation script launched.", flush=True)
                    except Exception as e:
                        print(f"Failed to launch navigation script: {e}", flush=True)
                elif points=='program lab':
                    try:
                        # Launch the navigation script
                        subprocess.Popen(["python3", "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/plc.py"])
                        print("Navigation script launched.", flush=True)
                    except Exception as e:
                        print(f"Failed to launch navigation script: {e}", flush=True)
                elif points == 'robotics lab' or points == 'robotic lab':
                    try:
                        # Launch the navigation script
                        subprocess.Popen(["python3", "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/robotics.py"])
                        print("Navigation script launched.", flush=True)
                    except Exception as e:
                        print(f"Failed to launch navigation script: {e}", flush=True)
                elif points=='design lab':
                    try:
                        # Launch the navigation script
                        subprocess.Popen(["python3", "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/design.py"])
                        print("Navigation script launched.", flush=True)
                    except Exception as e:
                        print(f"Failed to launch navigation script: {e}", flush=True)
                nav = 1


        # Publish the message
        if publish_msg == 1:
            pub.publish(msg)
            trigger = 1
            continue
        if stop_command == 1:
            pub.publish(msg)
            stop_command = 0
            continue
        if stop_chat == 1:
            continue
        if nav == 1:
            nav=0
            continue

        # TODO: UNCOMMENT
        # Clear previous response chunks
        interrupt_event.clear()
        while not response_queue.empty():
            response_queue.get()

        start_threads()

        # Print streamed response
        print("Assistant: ", end="", flush=True)
        while True:
            try:
                chunk = response_queue.get(timeout=30)  # Get chunk from queue
                if chunk is None:  # Stop printing if get to end of stream
                    break
                print(chunk, end="", flush=True)        # Print chunk as it gets updated
            except queue.Empty:
                if interrupt_event.is_set():
                    print("\n[Response interrupted]")
                else:
                    print("\nTimeout waiting for response")
                break
        print("\n")
