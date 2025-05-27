import whisper
import sounddevice as sd
import numpy as np

# Load the Whisper model
model = whisper.load_model("tiny")  # You can use "tiny", "base", "small", "medium", or "large"

# Audio parameters
SAMPLE_RATE = 16000  # Whisper model requires 16 kHz audio
DURATION = 5         # Duration of each audio capture in seconds

def callback(indata, frames, time, status):
    if status:
        print(f"Status: {status}")
    audio_data = indata[:, 0]  # Extract mono audio
    audio_data = (audio_data * 32767).astype(np.int16)  # Convert to 16-bit PCM format
    print("Processing audio...")
    try:
        result = model.transcribe(audio_data, fp16=False)
        print(f"Recognized text: {result['text']}")
    except:
        pass
    

def main():
    print("Listening... (Press Ctrl+C to stop)")
    try:
        with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, callback=callback):
            sd.sleep(int(DURATION * 1000))  # Wait for the audio to be processed
    except KeyboardInterrupt:
        print("\nStopped.")

if __name__ == "__main__":
    main()


# import speech_recognition as sr
# import pyttsx3
# import os

# listener = sr.Recognizer()
# engine = pyttsx3.init()    #Start tts engine

# #Set Voice Type
# voices = engine.getProperty('voices')
# engine.setProperty('voice', voices[1].id)

# #Set Voice Rate
# rate = engine.getProperty('rate')   # getting details of current speaking rate
# print (rate)                        #printing current voice rate
# engine.setProperty('rate', 125)     # setting up new voice rate

# engine.say("Good day, how may I help you today?")
# engine.runAndWait()

# with sr.Microphone() as source:
#     while True:
#         try:
#             # os.system('clear')
#             print(listener.energy_threshold)
#             print('listening')
#             # listener.adjust_for_ambient_noise(source, duration=1)
#             print(listener.energy_threshold)

#             # listener.energy_threshold = 300  # Adjust based on your environment
#             voice = listener.listen(source)

#             command = listener.recognize_google(voice)
#             print(command)
#             print("Completed")

            
#         except sr.UnknownValueError:
#             print("Could not understand audio.")
#         except sr.RequestError as e:
#             print(f"Error with Google API: {e}")

# moveBindings = {
#     'forward': (1, 0, 0, 0),
#     'left': (0, 0, 0, 1),
#     'right': (0, 0, 0, -1),
#     'reverse': (-1, 0, 0, 0),
#     'stop': (0, 0, 0, 0),
# }

# command = "i am stop"  # Example command
# command = command.split()  # Split speech into list of words

# for item in command:  # Iterate through each word in the command list
#     if isinstance(item, str):  # Check if the item is a string (just to be sure)
#         if item in moveBindings:  # Check if the word is a key in the dictionary
#             print(f"Command '{item}' found. Corresponding action: {moveBindings[item]}")
#         else:
#             print(f"Command '{item}' not found in moveBindings.")
#     else:
#         print(f"Unexpected data type: {item}")
