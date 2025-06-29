# Copyright 2011 Brown University Robotics.
# Copyright 2017 Open Source Robotics Foundation, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import sys
import threading
import os

import geometry_msgs.msg
import rclpy

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

#Speech Libraries
import speech_recognition as sr
import pyttsx3
import pyaudio

# msg = """
# This node takes keypresses from the keyboard and publishes them
# as Twist/TwistStamped messages. It works best with a US keyboard layout.
# ---------------------------
# Moving around:
#    u    i    o
#    j    k    l
#    m    ,    .

# For Holonomic mode (strafing), hold down the shift key:
# ---------------------------
#    U    I    O
#    J    K    L
#    M    <    >

# t : up (+z)
# b : down (-z)

# anything else : stop

# q/z : increase/decrease max speeds by 10%
# w/x : increase/decrease only linear speed by 10%
# e/c : increase/decrease only angular speed by 10%

# CTRL-C to quit
# """

# moveBindings = {
#     'i': (1, 0, 0, 0),
#     'o': (1, 0, 0, -1),
#     'j': (0, 0, 0, 1),
#     'l': (0, 0, 0, -1),
#     'u': (1, 0, 0, 1),
#     ',': (-1, 0, 0, 0),
#     '.': (-1, 0, 0, 1),
#     'm': (-1, 0, 0, -1),
#     'O': (1, -1, 0, 0),
#     'I': (1, 0, 0, 0),
#     'J': (0, 1, 0, 0),
#     'L': (0, -1, 0, 0),
#     'U': (1, 1, 0, 0),
#     '<': (-1, 0, 0, 0),
#     '>': (-1, -1, 0, 0),
#     'M': (-1, 1, 0, 0),
#     't': (0, 0, 1, 0),
#     'b': (0, 0, -1, 0),
# }

# speedBindings = {
#     'q': (1.1, 1.1),
#     'z': (.9, .9),
#     'w': (1.1, 1),
#     'x': (.9, 1),
#     'e': (1, 1.1),
#     'c': (1, .9),
# }

msg = """
Recognizing voice and publishing to Twist!
---------------------------
Moving around:
   Move Forward: Forward
   Move Backward: Reverse
   Turn Left: Left
   Turn Right: Right
   Stop: Stop


   Increase/decrease max speeds by 10%: Speed Up/Slow Down
   Increase/decrease only linear speed by 10%: Move Faster/Slower
   Increase/decrease only angular speed by 10%: Turn Faster/Slower
   
Say Ross + "Command"
---------------------------

CTRL-C to quit
"""

moveBindings = {
		'forward':(1,0,0,0),
		'left':(0,0,0,1),
		'right':(0,0,0,-1),
		'reverse':(-1,0,0,0),
		'stop':(0,0,0,0),
	       }

speedBindings={
		'speed up':(1.1,1.1),
		'slow down':(.9,.9),
		'move faster':(1.1,1),
		'move slower':(.9,1),
		'turn faster':(1,1.1),
		'turn slower':(1,.9),
	      }

listener = sr.Recognizer() #Start sr module
engine = pyttsx3.init()    #Start tts engine

#Set Voice Type
voices = engine.getProperty('voices')
engine.setProperty('voice', voices[0].id)

#Set Voice Rate
rate = engine.getProperty('rate')     # getting details of current speaking rate
print(f"Current Voice Rate: {rate}")  # printing current voice rate
engine.setProperty('rate', 150)       # setting up new voice rate

def command(settings, speed, turn):
    if sys.platform != 'win32':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    
    
    with sr.Microphone() as source:
        # os.system('clear')
        print(msg)
        print(vels(speed, turn))
        print("\nListening")
        # listener.adjust_for_ambient_noise(source, duration=3)
        listener.energy_threshold = 250
        listener.dynamic_energy_threshold = False  # Disable automatic adjustment
        audio = listener.listen(source)
        command = listener.recognize_google(audio)
        print("Acknowledged")
        command = command.lower()
        if 'alexa' in command:#change back to ross
            print("Entered")
            input_to_speech(command)
            return command
        else:
            print("No.....")

def input_to_speech(command):
    engine.say(command)
    engine.runAndWait()
    print(command)

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def vels(speed, turn):
    return 'currently:\tspeed %s\tturn %s ' % (speed, turn)


def main():
    settings = saveTerminalSettings()

    rclpy.init()

    node = rclpy.create_node('voice_control')

    # parameters
    stamped = node.declare_parameter('stamped', False).value
    frame_id = node.declare_parameter('frame_id', '').value
    if not stamped and frame_id:
        raise Exception("'frame_id' can only be set when 'stamped' is True")

    if stamped:
        TwistMsg = geometry_msgs.msg.TwistStamped
    else:
        TwistMsg = geometry_msgs.msg.Twist

    pub = node.create_publisher(TwistMsg, 'cmd_vel', 10)

    spinner = threading.Thread(target=rclpy.spin, args=(node,))
    spinner.start()

    speed = 0.5
    turn = 1.0
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0

    twist_msg = TwistMsg()

    if stamped:
        twist = twist_msg.twist
        twist_msg.header.stamp = node.get_clock().now().to_msg()
        twist_msg.header.frame_id = frame_id
    else:
        twist = twist_msg

    try:
        print("Initiated")

        #Greeting
        input_to_speech("Good day, how may I help you today?")

        order = 'Stop'
        while True:
            try:
                order = command(settings, speed, turn)
                order = order.split()
            except:
                if order != 'Stop':
                    order = order
                else:
                    order = 'Stop'
            # order = command(settings)
            # order = order.split()
            print(order)
            if order != None:
                for item in order:
                    if item in moveBindings:
                        x = moveBindings[item][0]
                        y = moveBindings[item][1]
                        z = moveBindings[item][2]
                        th = moveBindings[item][3]
                    elif item in speedBindings.keys():
                        speed = speed * speedBindings[item][0]
                        turn = turn * speedBindings[item][1]

                        print(vels(speed, turn))
                        if (status == 14):
                            print(msg)
                        status = (status + 1) % 15
                    else:
                        x = 0.0
                        y = 0.0
                        z = 0.0
                        th = 0.0
                        if (command == 'exit'):
                            break

            if stamped:
                twist_msg.header.stamp = node.get_clock().now().to_msg()

            twist.linear.x = x * speed
            twist.linear.y = y * speed
            twist.linear.z = z * speed
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = th * turn
            pub.publish(twist_msg)

    except sr.UnknownValueError:
        print("Speech Recognition could not understand audio.")
    except sr.RequestError as e:
        print(f"Could not request results from Google Speech Recognition service; {e}")
    except Exception as e:
        print(f"An unexpected error occurred: {e}")

    finally:
        if stamped:
            twist_msg.header.stamp = node.get_clock().now().to_msg()

        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist_msg)
        rclpy.shutdown()
        spinner.join()

        restoreTerminalSettings(settings)

if __name__ == '__main__':
    main()