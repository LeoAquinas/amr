# Can use to compare speed
# Speed of using model is very bad compared to just normal google

import speech_recognition as sr
import pyttsx3
import os
import whisper

listener = sr.Recognizer()
engine = pyttsx3.init()    #Start tts engine

#Set Voice Type
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[1].id)

#Set Voice Rate
rate = engine.getProperty('rate')   # getting details of current speaking rate
print (rate)                        #printing current voice rate
engine.setProperty('rate', 125)     # setting up new voice rate

engine.say("Good day, how may I help you today?")
engine.runAndWait()

model = whisper.load_model("tiny.en")

with sr.Microphone() as source:
    while True:
        try:
            # os.system('clear')
            print(listener.energy_threshold)
            print('listening')
            # listener.adjust_for_ambient_noise(source, duration=3)
            listener.energy_threshold = 250
            listener.dynamic_energy_threshold = False  # Disable automatic adjustment

            if listener.energy_threshold < 30:
                continue

            # listener.energy_threshold = 300  # Adjust based on your environment
            voice = listener.listen(source)

            command = listener.recognize_whisper(voice)
            # command = model.transcribe(source)
            print(command)
            print("Completed")

            
        except sr.UnknownValueError:
            print("Could not understand audio.")
        except sr.RequestError as e:
            print(f"Error with Google API: {e}")

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
