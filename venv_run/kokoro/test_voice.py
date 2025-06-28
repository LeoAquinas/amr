#!/home/jetson/venvs/kokoro/bin/python3

import speech_recognition as sr
import ollama
from faster_whisper import WhisperModel
import tempfile
import os
import threading
import queue

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
model = WhisperModel("tiny", device="cpu", compute_type="int8")
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
forward_command_list = ['forward', 'front', 'advance', 'go forth', 'onward']
# To move backwards
backward_command_list = ['reverse', 'back', 'retreat', 'withdraw', 'pull back', 'back up']
# To move left
left_command_list = ['left']
# To move right
right_command_list = ['right']
# To stop
stop_command_list = ['stop', 'halt']

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
        print(f"Audio playback error: {e}")

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
                    print(f"Final TTS Error: {e}")
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
                    play_audio(samples, sample_rate)

        except Exception as e:
            print(f"TTS Error: {e}")


def speech_to_text():
    recognizer = sr.Recognizer()
    
    with sr.Microphone() as source:
        print("Adjusting for ambient noise...")
        recognizer.adjust_for_ambient_noise(source, duration=1)
        print("Listening... (speak now)")
        
        try:
            # Capture audio using speech_recognition
            audio = recognizer.listen(source, timeout=5)
            
            # Save audio to temporary file
            with tempfile.NamedTemporaryFile(suffix=".wav", delete=False) as fp:
                fp.write(audio.get_wav_data())
                temp_path = fp.name
            
            # Transcribe with Whisper
            segments, _ = model.transcribe(temp_path)
            text = " ".join([segment.text for segment in segments])
            
            # Clean up temp file
            os.remove(temp_path)
            
            return text
            
        except sr.WaitTimeoutError:
            return "No speech detected"
        except Exception as e:
            return f"Error: {str(e)}"

# def response():
def response(input):
    try:
        # Start streaming response
        stream = ollama.generate(
            model='gemma3:1b',
            # prompt='Why is the sky blue?',
            prompt =input,
            options={'temperature': 0.3, 'num_predict': 200},   # Can consider removing this
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

    while True:
        # TODO: UNCOMMENT
        input("Press Enter to speak...")
        # Speech to text with ollama
        result = speech_to_text()
        print(f"You said: {result}\n")

        # Retry response if invalid speech input
        if "No speech detected" in result or "Error:" in result:
            continue

        # # Clean commands, changing to lower case to compare with list
        # # cleaned_result = result.strip().lower()
        # cleaned_result = 'forward'

        # # Seperate result using regex
        # # \w+[\w']* matches:
        # # \w+: One or more word characters (letters/digits)
        # # [\w']*: Optional continuation of word characters or apostrophes (for contractions like "don't")
        # # [,.?] matches individual punctuation marks (,, ., ?)
        # separated_cleaned_result = re.findall(r"\w+[\w']*|[,.?]", cleaned_result)
        # print(separated_cleaned_result)

        # # Create String message
        # msg = String()

        # for command in separated_cleaned_result:
        #     if command in forward_command_list:
        #         msg.data = 'forward'
        #     elif command in backward_command_list:
        #         msg.data = 'backward'
        #     elif command in left_command_list:
        #         msg.data = 'left'
        #     elif command in right_command_list:
        #         msg.data = 'right'
        #     elif command in stop_command_list:
        #         msg.data = 'stop'

        # # Publish the message
        # pub.publish(msg)
        # print(f"Published: {msg}")

        # continue

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