import customtkinter as ctk
from PIL import Image, ImageTk
import psutil
import subprocess

# === Globals to track processes and states ===
robot_process = None

mic_process = None

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

def on_button1_click():
    print("Button 1 clicked!")

def on_button2_click():
    print("Button 2 clicked!")

def on_button3_click():
    print("Button 3 clicked!")

def on_robot_click():
    global robot_process
    if robot_process is None or robot_process.poll() is not None:
        robot_process = start_launch('ros2 launch amr test_launch_sim.launch.py')
        print("Robot launch started.")
    else:
        kill_process_tree(robot_process.pid)
        robot_process = None
        print("Robot launch stopped.")

    # subprocess.Popen(
    #     ["ros2", "launch", "amr", "test_launch_sim.launch.py"],
    #     stdout=subprocess.PIPE,
    #     stderr=subprocess.PIPE
    # )

def on_mic_click():
    global mic_process
    if mic_process is None or mic_process.poll() is not None:
        mic_process = start_launch("ros2 launch your_mic_launch_file.launch.py")
        print("Mic launch started.")
    else:
        kill_process_tree(mic_process.pid)
        mic_process = None
        print("Mic launch stopped.")




# === Setup CTk ===
ctk.set_appearance_mode("Dark")  # Start in dark mode
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
    print("Icon images not found! Please ensure 'moon.png' and 'sun.png' are in the script folder.")
    moon_img = sun_img = None

# === Load robot and mic icons ===
try:
    robot_icon = Image.open("/home/jetson/Desktop/icons8-robot-50.png").resize((50, 50))
    mic_icon = Image.open("/home/jetson/Desktop/icons8-microphone-50.png").resize((50, 50))
    robot_img = ImageTk.PhotoImage(robot_icon)
    mic_img = ImageTk.PhotoImage(mic_icon)
except FileNotFoundError:
    print("Robot or mic icon images not found! Please check paths.")
    robot_img = mic_img = None

# === Toggle Theme Function ===
def toggle_theme():
    current = ctk.get_appearance_mode()
    new_mode = "Light" if current == "Dark" else "Dark"
    ctk.set_appearance_mode(new_mode)
    if new_mode == "Light":
        icon_button.configure(image=moon_img)
    else:
        icon_button.configure(image=sun_img)

# === Title ===
title_label = ctk.CTkLabel(app, text="Your Title Here", font=("Arial", 24))
title_label.grid(row=0, column=0, columnspan=3, pady=(10, 5), sticky="n")

# === Dark Mode Icon Button (Top-right corner) ===
icon_button = ctk.CTkButton(app, width=30, height=30, text="", command=toggle_theme, image=sun_img)
icon_button.grid(row=0, column=2, sticky="ne", padx=(0, 20), pady=(10, 0))

# === Left Panel ===
left_frame = ctk.CTkFrame(app, corner_radius=0)
left_frame.grid(row=1, column=0, sticky="nsew", padx=(10, 5), pady=10)

# Let the column inside left_frame expand horizontally
left_frame.grid_columnconfigure(0, weight=1)

# Configure rows for buttons, spacer, and bottom buttons
left_frame.grid_rowconfigure(0, weight=0)
left_frame.grid_rowconfigure(1, weight=0)
left_frame.grid_rowconfigure(2, weight=0)
left_frame.grid_rowconfigure(3, weight=1)
left_frame.grid_rowconfigure(4, weight=0)  # for bottom button frame

button1 = ctk.CTkButton(left_frame, text="Button 1", command=on_button1_click)
button1.grid(row=0, column=0, pady=(20, 10), padx=10, sticky="ew")

button2 = ctk.CTkButton(left_frame, text="Button 2", command=on_button2_click)
button2.grid(row=1, column=0, pady=10, padx=10, sticky="ew")

button3 = ctk.CTkButton(left_frame, text="Button 3", command=on_button3_click)
button3.grid(row=2, column=0, pady=10, padx=10, sticky="ew")

spacer = ctk.CTkFrame(left_frame, fg_color="transparent")
spacer.grid(row=3, column=0, sticky="nsew")

# Bottom buttons frame inside left_frame
bottom_button_frame = ctk.CTkFrame(left_frame, fg_color="transparent")
bottom_button_frame.grid(row=4, column=0, sticky="ew", padx=10, pady=(20, 10))

# Equal column expansion inside bottom_button_frame
bottom_button_frame.grid_columnconfigure(0, weight=1)
bottom_button_frame.grid_columnconfigure(1, weight=1)

# Robot button
robot_button = ctk.CTkButton(
    bottom_button_frame,
    width=50,
    height=50,
    text="",
    image=robot_img,
    command=on_robot_click
)
robot_button.grid(row=0, column=0, sticky="ew", padx=(0, 10))

# Mic button
mic_button = ctk.CTkButton(
    bottom_button_frame,
    width=50,
    height=50,
    text="",
    image=mic_img,
    command=on_mic_click
)
mic_button.grid(row=0, column=1, sticky="ew", padx=(10, 0))

# === Right Panel (Image Display) ===
right_frame = ctk.CTkFrame(app, corner_radius=0)
right_frame.grid(row=1, column=1, columnspan=2, sticky="nsew", padx=(5, 10), pady=10)
right_frame.grid_rowconfigure(0, weight=1)
right_frame.grid_columnconfigure(0, weight=1)

# Load and display image
try:
    pil_image = Image.open("/home/jetson/Desktop/official_pgm_files/rtabmap_lib.pgm")
    pil_image = pil_image.resize((500, 500))  # make image bigger
    img = ImageTk.PhotoImage(pil_image)
    image_label = ctk.CTkLabel(right_frame, image=img, text="")
    image_label.place(relx=0.5, rely=0.5, anchor="center")
except FileNotFoundError:
    image_label = ctk.CTkLabel(right_frame, text="Image not found")
    image_label.place(relx=0.5, rely=0.5, anchor="center")

# === Run App ===
app.mainloop()
