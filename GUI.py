import customtkinter as ctk
from PIL import Image, ImageTk
import psutil
import subprocess
import sys
import threading
import os
import signal
import time
import logging
import rclpy
from rclpy import logging as ros_logging
import re
import tkinter as tk


# === Redirect print output to Textbox ===
class TextRedirector(object):
    def __init__(self, text_widget):
        self.text_widget = text_widget

    def write(self, msg):
        # Queue GUI-safe update
        self.text_widget.after(0, self._write_to_box, msg)

    def _write_to_box(self, msg):
        try:
            # Temporarily make it editable
            self.text_widget.configure(state="normal")

            # Insert and scroll
            self.text_widget.insert("end", msg if msg.endswith('\n') else msg + '\n')
            self.text_widget.see("end")

            # Lock again
            self.text_widget.configure(state="disabled")

            # Force visual refresh
            self.text_widget.update_idletasks()
        except Exception as e:
            print(f"Log update error: {e}")

    def flush(self):
        pass  # Needed for compatibility

    def emit(self, record):
        log_message = self.format(record)
        self.write(log_message)  # Use the `write` method to display log messages

# Helper to launch a process and stream its output into the GUI
def launch_and_stream(cmd_list):
    """
    Launches the given command list as a subprocess and
    fires off two threads to read its stdout and stderr,
    sending every line into our redirector.
    Returns the Popen object.
    """
    proc = subprocess.Popen(
        cmd_list,
        stdin=subprocess.PIPE, 
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        bufsize=1,
        text=True,              # so we get strings, not bytes
        preexec_fn=os.setsid    # so we can kill entire group later
    )

    def reader(pipe):
        for line in iter(pipe.readline, ''):
            # strip trailing newline because write() handles it
            redirector.write(line.rstrip('\n'))
        pipe.close()

    # Start one thread for stdout and one for stderr
    threading.Thread(target=reader, args=(proc.stdout,), daemon=True).start()
    threading.Thread(target=reader, args=(proc.stderr,), daemon=True).start()

    return proc# Helper to launch a process and stream its output into the GUI

# === Globals to track processes and states ===
robot_process = None
mic_process = None
motor_processes = []  # To track multiple motor script processes

# === Button Functions ===
def start_launch(cmd):
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def kill_process_tree(pid):
    try:
        parent = psutil.Process(pid)
        children = parent.children(recursive=True)
        for child in children:
            child.terminate()
        parent.terminate()
    except psutil.NoSuchProcess:
        pass

def run_simple_commander(script_name):
    full_command = f"python3 {script_name}"
    return subprocess.Popen(
        ["/bin/bash", "-c", full_command],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE
    )

def on_button1_click():
    print("Navigating to Robotics Lab!")
    print(time.time())
    launch_and_stream([
         "python",
         "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/robotics.py"
     ])
    
def on_button2_click():
    print("Navigating to PLC Lab!")
    print(time.time())
    launch_and_stream([
         "python",
         "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/plc.py"
     ])
    
def on_button3_click():
    print("Navigating to Design Lab!")
    print(time.time())
    launch_and_stream([
         "python",
         "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/design.py"
     ])
    
def on_button4_click():
    print("Navigating to Machine Vision Lab!")
    print(time.time())
    launch_and_stream([
         "python",
         "/home/jetson/agv/src/amr/launch/simple_commander/lab_test/mvi.py"
     ])
    
def on_robot_click():
    global robot_process
    if robot_process is None or robot_process.poll() is not None:
        robot_process = subprocess.Popen(
            ["ros2", "launch", "launch_amr", "demo_launch.launch.py"],
            stdout=subprocess.DEVNULL,      # drop all normal output
            stderr=subprocess.DEVNULL,      # drop all error output too
            preexec_fn=os.setsid,
            text=True
        )
        print("Robot launch started.")
    else:
        kill_process_tree(robot_process.pid)
        robot_process = None
        print("Robot launch stopped.")

def on_mic_click():
    # global mic_process
    # if mic_process is None or mic_process.poll() is not None:
    #     python_venv = "/home/jetson/venvs/kokoro/bin/python"
    #     script = "/home/jetson/agv/src/amr/voice_packages/test_voice.py"
    #     mic_process = subprocess.Popen([python_venv, script])
    #     print("Mic launch started.")
    # else:
    #     kill_process_tree(mic_process.pid)
    #     mic_process = None
    #     print("Mic launch stopped.")
    global mic_process
    script = "/home/jetson/agv/src/amr/voice_packages/kokoro_launcher.py"
    if mic_process is None or mic_process.poll() is not None:
        # Ensure launcher is executable and has a proper shebang
        mic_process = launch_and_stream([
             "python",
             "/home/jetson/agv/src/amr/voice_packages/kokoro_launcher.py"
         ])
        print("Mic launch started.")
    else:
        try:
            # send SIGTERM to the whole group
            pgid = os.getpgid(mic_process.pid)
            os.killpg(pgid, signal.SIGTERM)
            time.sleep(0.5)
            os.killpg(pgid, signal.SIGKILL)
        except Exception:
            pass
        mic_process = None
        print("Mic launch stopped.")

def on_motor_click():
    global motor_processes
    if motor_processes:  # If already running, stop all
        for proc in motor_processes:
            if proc.poll() is None:  # Still running
                kill_process_tree(proc.pid)
        motor_processes = []
        print("Motor scripts stopped.")
    else:  # Start both motor scripts
        p1 = launch_and_stream(['python', '/home/jetson/agv/src/amr/voice_packages/voice_subscriber.py'])
        p2 = launch_and_stream(['python', '/home/jetson/agv/src/amr/serial_test/serial_test/serial_test.py'])
        motor_processes = [p1, p2]
        print("Motor scripts started.")

# === Setup CTk ===
ctk.set_appearance_mode("Dark")
ctk.set_default_color_theme("blue")

app = ctk.CTk()
app.title("CustomTkinter GUI Layout")
app.geometry("900x600")

# === Grid Setup ===
app.grid_rowconfigure(1, weight=1)
app.grid_columnconfigure(0, weight=1)
app.grid_columnconfigure(1, weight=3)

# === Load Icons ===
try:
    moon_icon = Image.open("/home/jetson/Desktop/icons8-moon-50.png").resize((24, 24))
    sun_icon = Image.open("/home/jetson/Desktop/icons8-sun-50.png").resize((24, 24))
    moon_img = ImageTk.PhotoImage(moon_icon)
    sun_img = ImageTk.PhotoImage(sun_icon)
except FileNotFoundError:
    print("Icon images not found!")
    moon_img = sun_img = None

try:
    robot_icon = Image.open("/home/jetson/Desktop/icons8-robot-50.png").resize((50, 50))
    mic_icon = Image.open("/home/jetson/Desktop/icons8-microphone-50.png").resize((50, 50))
    motor_icon = Image.open("/home/jetson/Desktop/icons8-wheel-50.png").resize((50, 50))
    robot_img = ImageTk.PhotoImage(robot_icon)
    mic_img = ImageTk.PhotoImage(mic_icon)
    motor_img = ImageTk.PhotoImage(motor_icon)
except FileNotFoundError:
    print("Robot or mic icon images not found!")
    robot_img = mic_img = motor_img = None

# === Toggle Theme ===
def toggle_theme():
    current = ctk.get_appearance_mode()
    new_mode = "Light" if current == "Dark" else "Dark"
    ctk.set_appearance_mode(new_mode)
    icon_button.configure(image=moon_img if new_mode == "Light" else sun_img)

def clear_log():
    log_box.configure(state="normal")
    log_box.delete("1.0", "end")
    log_box.configure(state="disabled")

def enter_command():
    if mic_process and mic_process.stdin:
        try:
            mic_process.stdin.write("\n")
            mic_process.stdin.flush()
        except BrokenPipeError:
            print("Mic process stdin closed.")
    else:
        print("Mic is not running.")
    
# === Title ===
title_label = ctk.CTkLabel(app, text="Your Title Here", font=("Arial", 24))
title_label.grid(row=0, column=0, columnspan=3, pady=(10, 5), sticky="n")

# === Dark Mode Toggle Button ===
icon_button = ctk.CTkButton(app, width=30, height=30, text="", command=toggle_theme, image=sun_img)
icon_button.grid(row=0, column=2, sticky="ne", padx=(0, 20), pady=(10, 0))

# === Left Panel ===
left_frame = ctk.CTkFrame(app, corner_radius=0)
left_frame.grid(row=1, column=0, sticky="nsew", padx=(10, 5), pady=10)
left_frame.grid_columnconfigure(0, weight=1)

# Configure rows
left_frame.grid_rowconfigure(0, weight=0)  # Button 1
left_frame.grid_rowconfigure(1, weight=0)  # Button 2
left_frame.grid_rowconfigure(2, weight=0)  # Button 3
left_frame.grid_rowconfigure(3, weight=0)  # Button 4
left_frame.grid_rowconfigure(4, weight=1)  # Spacer
left_frame.grid_rowconfigure(5, weight=0)  # Bottom buttons

# Buttons
button1 = ctk.CTkButton(left_frame, text="Robotics Lab", command=on_button1_click)
button1.grid(row=0, column=0, pady=(20, 10), padx=10, sticky="ew")

button2 = ctk.CTkButton(left_frame, text="PLC Lab", command=on_button2_click)
button2.grid(row=1, column=0, pady=10, padx=10, sticky="ew")

button3 = ctk.CTkButton(left_frame, text="Design Lab", command=on_button3_click)
button3.grid(row=2, column=0, pady=10, padx=10, sticky="ew")

button4 = ctk.CTkButton(left_frame, text="Machine Vision Lab", command=on_button4_click)
button4.grid(row=3, column=0, pady=10, padx=10, sticky="ew")

log_box = ctk.CTkTextbox(
    left_frame,
    width=200,
    height=200,
    fg_color="#111111",  # solid background
    text_color="white"
)
log_box.grid(row=4, column=0, columnspan=2, pady=10, padx=10, sticky="ew")
# make it read-only by default:
log_box.configure(state="disabled")

# Frame below log box for small buttons
log_button_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
log_button_frame.grid(row=4, column=0, sticky="se", padx=10, pady=(0, 10))
log_button_frame.grid_columnconfigure((0, 1), weight=1)

# Clear Log button
clear_log_button = ctk.CTkButton(log_button_frame, text="Clear Log", width=10, height=25, command=lambda: clear_log())
clear_log_button.grid(row=0, column=0, padx=(0, 5))

# Enter Command button
enter_button = ctk.CTkButton(log_button_frame, text="Enter", width=10, height=25, command=lambda: enter_command())
enter_button.grid(row=0, column=1, padx=(5, 0))

# === Redirect terminal output to scrollable frame ===
redirector = TextRedirector(log_box)
sys.stdout = redirector
sys.stderr = redirector

# Bottom Buttons Frame
bottom_button_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
bottom_button_frame.grid(row=5, column=0, sticky="ew", padx=10, pady=(20, 10))
bottom_button_frame.grid_columnconfigure(0, weight=1)
bottom_button_frame.grid_columnconfigure(1, weight=1)

# Motor Button (centered above Robot and Mic buttons)
motor_button = ctk.CTkButton(bottom_button_frame, width=50, height=50, text="", image=motor_img, command=on_motor_click)
motor_button.grid(row=0, column=0, columnspan=2, pady=(0, 10), sticky="ew")

# Robot and Mic buttons below motor button
robot_button = ctk.CTkButton(bottom_button_frame, width=50, height=50, text="", image=robot_img, command=on_robot_click)
robot_button.grid(row=1, column=0, sticky="ew", padx=(0, 10))

mic_button = ctk.CTkButton(bottom_button_frame, width=50, height=50, text="", image=mic_img, command=on_mic_click)
mic_button.grid(row=1, column=1, sticky="ew", padx=(10, 0))

# === Right Panel ===
right_frame = ctk.CTkFrame(app, corner_radius=0)
right_frame.grid(row=1, column=1, columnspan=2, sticky="nsew", padx=(5, 10), pady=10)
right_frame.grid_rowconfigure(0, weight=1)
right_frame.grid_columnconfigure(0, weight=1)

# Display Image
try:
    pil_image = Image.open("/home/jetson/Desktop/labeled_corridor.png")
    pil_image = pil_image.resize((500, 500))
    img = ImageTk.PhotoImage(pil_image)
    image_label = ctk.CTkLabel(right_frame, image=img, text="")
    image_label.place(relx=0.5, rely=0.5, anchor="center")
except FileNotFoundError:
    image_label = ctk.CTkLabel(right_frame, text="Image not found")
    image_label.place(relx=0.5, rely=0.5, anchor="center")


# === Run App ===
app.mainloop()
