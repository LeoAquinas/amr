import subprocess
import sys
import os

# Define the path to your virtual environment
venv_path = "/home/jetson/venvs/kokoro"

# Construct the appropriate activation command based on the OS
if sys.platform == 'win32':
    activate_script = os.path.join(venv_path, 'Scripts', 'activate.bat')
    python_executable = os.path.join(venv_path, 'Scripts', 'python.exe')
else:
    activate_script = os.path.join(venv_path, 'bin', 'activate')
    python_executable = os.path.join(venv_path, 'bin', 'python')

# Define the script you want to run within the virtual environment
script_to_run = "/home/jetson/agv/src/amr/venv_run/kokoro/test_voice.py"

# Construct the command to activate venv and run the script
command = f"source {activate_script} && {python_executable} {script_to_run}"

# Execute the command in a subprocess
subprocess.run(command, shell=True, executable="/bin/bash")
