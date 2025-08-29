# README
This is a repository for a final year project pertaining to the HRI system of an AMR with the functionalities of:
  1. Autonomous Navigation
  2. Voice/Speech Control
  3. Teleoperation Control
  4. Touchpad Inputs
  5. Custom YOLO Nav2 package for obstacle avoidance enhancement
**To run**:
- Jetson GUI disabled by default due to usage of Nomachine. To enable:
    - `sudo systemctl enable gdm3 --now`. This would enable the GUI on the Jetson such that can navigate like indows and linux. Otherwise everything would be on terminal
    - If want to connect using Nomachine instead, need to install Nomachine on own device for connection. If trying to connect to Jetson using this method, need to use mobile data as APU WIFI blocks connections.
- There are 2 GUI available in the Jetson.
    - GUI.py is with the availability of voice commands, but custom yolo package is disabled
    - GUI2.py has custom yolopackage enabled, but voice commands disabled
  **Take note:**
    a. When initiating the AMR, make sure all motors are off before running the motor serial scripts as when serial signals are etablished with Arduino, the motors would backdrive
    b. When starting the AMR motors, make sure to block the back as sometimes when the jumper wires of the Arduino are loose or there is noise, the motors would backdrive. For this, please follow the steps of 1. Turn on the Arduino. 2. Turn on the controller. 3. Block the back. 4. Turn on the motors. **If backdrive problem persists for long time, wiggle the wires a bit**

## Overall Packages in this Github Repo
| Packages | Usage |
| --- | --- |
|`YOLO`|For YOLO detections and publication in ROS2 and for cropping of pointclouds for custom plugin|
|`launch`|Main launch files for AMR functionalities|
|`my_bot`|Package containing robot bringup and URDFs|
|`serial_test`|Package for establishing serial communication between Jetson and Arduino. Used to convert /cmd_vel topics into serial commands. **Before running this, make sure motors are off**|
|`voice_packages`|Scripts for voice communication for the AMR|
|`yolo_costmap_package`|Custom Nav2 plugin package that updates costmaps using YOLO detections for enhanced obstacle avoidance|
|`GUI.py`|GUI for AMR operations. Able to use voice functionality but custom plugin is disabled due to hardware constraints|
|`GUI2.py`|GUI for AAMR operations. Custom plugin enabled but voice functionality disabled due to hardware constraints|
|`arduino_control`|Arduino files for movement|
|`camera_info`|Extra evaluation script for OpenLORIS dataset, can ignore for functionality|
|`imu_fusion`|Extra evaluation script for OpenLORIS dataset, can ignore for functionality|
|`odom_rotator`|Extra evaluation script for OpenLORIS dataset, can ignore for functionality|
|`voice_control`|Developmental scripts for voice control functionality, can use as reference but useless for funtionality|
|`files_from_other_packages`|Files for other external packages in case the packages are accidentally deleted. Already setup in Jetson so can ignore, but if need to reset those packages, can use back the configs in these files|

## Overall Packages in Jetson
| Packages | Usage |
|---|---|
|`amr`|Main Robot Package|
|`rtab-map`|Robot Mapping|
|`ORBSLAM3`|To Provide Visual Odometry|
|`sllidar`|Package for LiDAR Functionality|
|`realsense-ros`|Package for IntelRealsense Camera|
|`robot_localization`|Package for Unscented Kalman Filter|


## Usage
Clone repository:
```
git clone https://github.com/LeoAquinas/amr.git
```
Setup Dependencies
### RTAB-Map
Clone from [source](https://github.com/introlab/rtabmap_ros/tree/ros2)
Install extra dependency:
```
sudo apt install ros-${ROS_DISTRO}-image-transport-plugins
```


### ORBSLAM3
**Due to hardware limitations this has been removed from the implementation, however if able to include this in Kalman Filter, robot odometry can improve**
Install ORBSLAM3 dependencies
  ##### Initial Dependencies:
  ```
  sudo apt update
  sudo apt-get install build-essential
  sudo apt-get install cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
  sudo apt-get install python-dev python-numpy libtbb2 libtbb-dev libjpeg-dev libpng-dev libtiff-dev libdc1394-22-dev libjasper-dev
  sudo apt-get install libglew-dev libboost-all-dev libssl-dev
  sudo apt install libeigen3-dev
  ```
  ##### OpenCV (Use 4.2.0)
  ```
  git clone https://github.com/opencv/opencv.git
  cd opencv
  git checkout 4.2.0
  mkdir build
  cd build
  cmake -D CMAKE_BUILD_TYPE=Release -D WITH_CUDA=OFF -D CMAKE_INSTALL_PREFIX=/usr/local ..
  make -j 1
  sudo make install
  ```
  ##### Pangolin
  ```
  git clone https://github.com/stevenlovegrove/Pangolin.git
  cd Pangolin 
  mkdir build 
  cd build 
  cmake .. -D CMAKE_BUILD_TYPE=Release 
  make -j 1 
  sudo make install
  ```
  ##### Install ORBSLAM3
  ```
  git clone https://github.com/zang09/ORB-SLAM3-STEREO-FIXED.git ORB_SLAM3
  cd ORB_SLAM3
  chmod +x build.sh
  ```
  Change [make -j](https://github.com/zang09/ORB-SLAM3-STEREO-FIXED/blob/master/build.sh) to avoid freezing PC. Do for all in the file.
  ```
  make -j2
  ```
  ##### Install ORBSLAM3 ROS2 Publisher
  Follow [this](https://github.com/zang09/ORB_SLAM3_ROS2)
  ##### Replace src files
  Replace mono and stereo folder in src folder in zang09/ORB_SLAM3_ROS2 with folders found [here](https://github.com/LimJingXiang1226/ELA2.0_NAV/tree/main) in others
  ##### Update subscribed topics
  In [this](https://github.com/LimJingXiang1226/ELA2.0_NAV/blob/main/other/stereo/stereo-slam-node.cpp) file **(example used for stereo, if using other node, equivalent topics should be updated)**
      Update subscribed topics from camera
      ```
      left_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/infra1/image_rect_raw");
      right_sub = std::make_shared<message_filters::Subscriber<ImageMsg>>(this, "camera/camera/infra2/image_rect_raw");
      ```


**Troubleshooting**

**1. [Pangolin execution error](https://github.com/UZ-SLAMLab/ORB_SLAM3/issues/399):**
  ```
  ./Examples/Monocular/mono_euroc: error while loading shared libraries: libpango_windowing.so: cannot open shared object file: No such file or directory
  ```
  Run: 
  ```
  sudo ldconfig
  ```
**2. ORBSLAM3 black screen**\
  Try to move camera around and wait, sometimes the algorithm takes a while to initialize
**3. [Compilation Error](https://github.com/alsora/ros2-ORB_SLAM2/issues/8)**
  ```
  --- stderr: ros2_orbslam
  /usr/bin/ld: CMakeFiles/stereo.dir/src/stereo/stereo-slam-node.cpp.o: undefined reference to symbol '_ZN2cv23initUndistortRectifyMapERKNS_11_InputArrayES2_S2_S2_NS_5Size_IiEEiRKNS_12_OutputArrayES7_'
  /usr/bin/ld: /usr/local/lib/libopencv_calib3d.so.405: error adding symbols: DSO missing from command line
  collect2: error: ld returned 1 exit status
  make[2]: *** [CMakeFiles/stereo.dir/build.make:217: stereo] Error 1
  make[1]: *** [CMakeFiles/Makefile2:101: CMakeFiles/stereo.dir/all] Error 2
  make: *** [Makefile:160: all] Error 2
  ---
  Failed   <<< ros2_orbslam [2.13s, exited with code 2]
  Aborted  <<< opencv_tests [2.74s]
  ```
  If seen above error, it is due to [opencv error](https://github.com/alsora/ros2-ORB_SLAM2/issues/8#issuecomment-1461570970). 
  Add the opencv lib in the CMakeLists.txt

  ```
  find_package(OpenCV 4.0 QUIET)
  ... 
  ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM2 Pangolin OpenCV)
  ```
**4. Disabling visualization**
```
#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "stereo-slam-node.hpp"
#include "System.h"

int main(int argc, char **argv)
{
    if (argc < 4)
    {
        std::cerr << "\nUsage: ros2 run orbslam stereo path_to_vocabulary path_to_settings do_rectify" << std::endl;
        return 1;
    }

    rclcpp::init(argc, argv);

    bool visualization = false;
    ORB_SLAM3::System pSLAM(argv[1], argv[2], ORB_SLAM3::System::STEREO, visualization);

    auto node = std::make_shared<StereoSlamNode>(&pSLAM, argv[2], argv[3]);
    RCLCPP_INFO(node->get_logger(), "ORB-SLAM3 Node Started");

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
```
To dosable visualization. change visualize = false;

### rf2o
Clone package:
```
git clone git@github.com:Adlink-ROS/rf2o_laser_odometry.git
```

### robot_localization
Clone package:
```
git clone https://github.com/cra-ros-pkg/robot_localization
git clone git@github.com:ros-geographic-info/geographic_info.git
apt-get install libgeographic-dev
```

### sllidar_ros2
[Package](https://github.com/Slamtec/sllidar_ros2) for LiDAR
```
git clone https://github.com/Slamtec/sllidar_ros2.git
```

  **Troubleshooting**
  
  **1. [No LiDAR data published](https://github.com/Slamtec/sllidar_ros2/issues/46)
    ```
    I encountered the exact same error as you, but mine is with the A1M8 model. I changed scan_mode = LaunchConfiguration('scan_mode', default='Sensitivity') to scan_mode = LaunchConfiguration('scan_mode', default='Standard'). After 
    that, it launched successfully!!
    ```

### laser_filters
[Package](https://github.com/ros-perception/laser_filters) for filtering laser scans
Clone package:
```
git clone git@github.com:ros-perception/laser_filters.git
```
LiDAR used for AMR was a 360 deg RPLiDAR A1. The placement of the LiDAR had its side covered by the robot body. As such, package was used to disclude unwanted laser scans.

The [angular filter](https://github.com/ros-perception/laser_filters/blob/ros2/examples/angular_filter_example.launch.py) was used.
To control the angles, the [config file](https://github.com/ros-perception/laser_filters/blob/ros2/examples/angular_filter_example.yaml) was modified to the desired angles.

**Once used, scan topic would be ```/scan_filtered```


### IntelRealSense
To use the IntelRealSense camera in ROS2, 2 steps are required.
  #### 1. Setup librealsense on Jetson Orin Nano
  Follow [this](https://github.com/jetsonhacks/jetson-orin-librealsense) setup.
    -- Can just run given commands in terminal. After the setup, user should be able to connect and see through camera.

  ### 2. Install ROS2 Wrapper for IntelRealsense
  Follow [this](https://github.com/jetsonhacks/jetson-orin-librealsense)
    --- Start directly from step 3. Once done, should be able to publish topics to ROS2

  ### Making use in URDF
  To make use of camera data, the camera frames need to be linked to base_link
    -- Put [this](https://github.com/IntelRealSense/realsense-ros/blob/ros1-legacy/realsense2_description/urdf/test_d435_camera.urdf.xacro) in the urdf file of your robot
      ->Obtained from [this](https://github.com/IntelRealSense/realsense-ros) package

  ### Run
  Can make use of launch file
    ->Alternatively, can make use of ros2 run cmd like [here](https://github.com/IntelRealSense/realsense-ros/issues/2873)
  

  **Troubleshooting**
  1. [RGB image topic fails to publish](https://github.com/IntelRealSense/realsense-ros/issues/1726)
       Might be due to resolution. change resolution to
       ```
       # param_name : 'width, height, frame-rate'
       'rgb_camera.color_profile' : '480, 240, 15',
       'depth_module.depth_profile' : '480, 240, 15,
       ```
  2. power_line_frequency warning
       Due to default issue with ROS wrapper not able to detect RGB frequency
       ```
       # Manually set parameter in launch file
       'rgb_camera.power_line_frequency' : 1
       ```

### Nav2
**Make use of turtlebot emanual navigation package due to configurable param file**
  If local costmap does not show during nav, change param file:
    1. local costmap global frame -> map
    2. ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True  # Or false depending on situation
    3. Param file should be the one within the humble directory within tuetlebot3_navigation launch


### Voice Control
**Due to GPU usage for kokoro onnx, need to use onnxruntime-gpu**
**onnxruntime-gpu installed following [ultralytics documentation](https://docs.ultralytics.com/guides/nvidia-jetson/#install-onnxruntime-gpu)**
**Due to numpy version issue, kokoro-onnx will be installed in a virtual environment**
```
python3 -m venv ~/venvs/kokoro
source ~/venvs/kokoro/bin/activate

pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
```
Dependencies:
```
# Install compatible version of numpy for onnx
pip install numpy==1.23.5
pip install -U kokoro-onnx
pip install soundfile
pip install faster-whisper
pip install SpeechRecognition
pip install sounddevice
curl -fsSL https://ollama.com/install.sh | sh
deactivate
```
To make use of gpu for kokoro, refer to [this](https://www.reddit.com/r/LocalLLaMA/comments/1htwkba/comment/m5jm0ta/?utm_source=share&utm_medium=web3x&utm_name=web3xcss&utm_term=1&utm_content=share_button) post
  References:
  [faster-whisper](https://github.com/SYSTRAN/faster-whisper)
  [ollama](https://github.com/ollama/ollama-python)
  [ollama models](https://ollama.com/library)
  [kokoro-onnx](https://github.com/thewh1teagle/kokoro-onnx)


#### Usage
1. Run [this](https://github.com/LeoAquinas/amr/blob/main/voice_packages/kokoro_launcher.py) file to start venv launch to initiate voice processes
2. [This](https://github.com/LeoAquinas/amr/blob/main/voice_packages/test_voice.py) file is source code for voice, but cannot run stright due to dependency issues in environment
3. Run [this](https://github.com/LeoAquinas/amr/blob/main/voice_packages/voice_subscriber.py) to subscribe to voice section and publish teleop commands to ROS2 environment
  

## Custom YOLO Plugin for Nav2
**Consist of 3 independant parts that need to be run simultaneously for use**
  
### 1. [YOLO](https://github.com/LeoAquinas/amr/tree/main/YOLO/yolo_launcher)
Dependencies:
**Numpy Error**
**kokoro-onnx requires numpy version == 2.0.2**
**yolo requires numpy version < 2 (Using 1.24.1)**
**Would have conflicts in packages.**

To resolve: 
Dependencies are installed in a virtual environment and the yolo node is launched using a launch file.
**Install ultralytics following [this](https://docs.ultralytics.com/guides/nvidia-jetson/#start-with-native-installation)**
```
python3 -m venv ~/venvs/numpy1241
source ~/venvs/numpy1241/bin/activate

sudo apt update
sudo apt install python3-pip -y
pip install -U pip
pip install ultralytics

deactivate
```
**Restart system**
```
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torch-2.5.0a0+872d972e41.nv24.08-cp310-cp310-linux_aarch64.whl
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/torchvision-0.20.0a0+afc54f7-cp310-cp310-linux_aarch64.whl
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-keyring_1.1-1_all.deb
sudo dpkg -i cuda-keyring_1.1-1_all.deb
sudo apt-get update
sudo apt-get -y install libcusparselt0 libcusparselt-dev
pip install https://github.com/ultralytics/assets/releases/download/v0.0.0/onnxruntime_gpu-1.20.0-cp310-cp310-linux_aarch64.whl
```
**Edit yolo_launcher launch file path to suit Device**
To fully optimize yolo performance, make use of tensorrt
1st, we need to [install tensorrt](https://forums.developer.nvidia.com/t/jetson-orin-nano-developer-kit-jetpack-cuda-tensorflow-with-gpu-and-tensorrt/260678/6)
...
#This will be installed in root, so find the site-package and transfer it to the venv site package for numpy1241
sudo apt install python3-libnvinfer*
...
2nd we install onnx in venv
```
pip install onnx
```

### 2. [Pointcloud Filtering](https://github.com/LeoAquinas/amr/tree/main/YOLO/pointcloud_crop)
  Makes use of [cropbox filter]([https://github.com/PointCloudLibrary/pcl/blob/master/filters/src/crop_box.cpp](https://github.com/ros-perception/perception_pcl/blob/ros2/pcl_ros/src/pcl_ros/filters/crop_box.cpp))
  **Filter come from perception_pcl library but can use as is as it comes with ROS2**
  **Used as a called method in pointcloud filter implementation file**
  Depending on resolution of image used from camera, need to change the width and height in [this](https://github.com/LeoAquinas/amr/blob/babe59a62d190519c769a21c45283a62f8109727/YOLO/pointcloud_crop/src/pointcloud_crop.cpp#L73) file

### 3. [Costmap Plugin](https://github.com/LeoAquinas/amr/tree/main/yolo_costmap_package)
  C++ source code for Nav2 Plugin
  **Independant source code that is loaded into Nav2 through Nav2 param file**
  **Used through the inclusion of the plugin [here](https://github.com/LeoAquinas/amr/blob/babe59a62d190519c769a21c45283a62f8109727/launch/config/nav2_yolo_params.yaml#L184) and [here](https://github.com/LeoAquinas/amr/blob/babe59a62d190519c769a21c45283a62f8109727/launch/config/nav2_yolo_params.yaml#L261) for global and local costmaps respectively**


### Arduino
Install Arduino IDE

[Linux Distro](https://www.arduino.cc/en/Guide/Linux/)

[Windows Distro](https://www.arduino.cc/en/Guide/Windows/)

**Both versions can be used. The IDE is only to upload code to the Arduino and would have no impact on the final system outcome.**

Arduino Dependencies (recommended to install through Arduino IDE rather than cloning from source repository):
  1. [CytronMotorDrivers](https://docs.arduino.cc/libraries/cytron-motor-drivers-library/)
  2. [NewPing](https://docs.arduino.cc/libraries/newping/)
The CytronMotorDrivers library was used as the motor controllers for the AMR was the MDDS10. For other types of controllers, the Arduino .ino code would need to be updated for motor interfacing.

If port access is denied, try [this](https://support.arduino.cc/hc/en-us/articles/360016495679-Fix-port-access-on-Linux)
**During usage of Jetson port, usb might not be able to be detected. To enable usb detection by Jetson, need to install and build kernel driver for USB**
->[Kernel Source](https://developer.nvidia.com/embedded/jetson-linux-r3643)

#### Final Arduino code for movement of AMR
[This](https://github.com/LeoAquinas/amr/blob/main/arduino_control/updated_arduino_code.ino) is the final working Arduino code for the AMR through serial commands


### [Serial Test](https://github.com/LeoAquinas/amr/blob/main/serial_test/serial_test/serial_test.py)
**Conversion script to change /cmd_vel topics into serial commands for Arduino to start movement**



## Colcon Build
Normal build:
```
colcon build --symlink-install
```
**When building the packages, the device may freeze due to computational limitations. To resolve this, build using following command instead:**
```
export MAKEFLAGS="-j1"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --executor sequential
```



## Commands
### Standalone Executions
Launch robot bringup to start robot
```
ros2 launch amr test_launch_sim.launch.py 
```

Start LiDAR
```
ros2 launch sllidar_ros2 view_sllidar_a1_launch.py
```

Start Camera
```
ros2 launch realsense2_camera camera_launch.launch.py 
```

Start rf2o
```
ros2 launch rf2o_laser_odometry rf2o_laser_odometry.launch.py 
```

Start conversion of /cmd_vel topics into serial movement for Arduino
```
ros2 run serial_test serial_test
```

Start yolo detections and publish into ROS2 environment
```
ros2 launch yolo_launcher yolo_launcher
```

Start pointcloud cropping based on yolo detections
```
ros2 run pointcloud_crop pointcloud_crop
```

Start voice communication
```
python {path to file}/kokoro_launcher.py
```

Convert voice teleoperation commands to /cmd_vel topics
```
python {path to file}/voice_subscriber.py
```

### Overall Execution
Launch robot bringup to start robot
```
ros2 launch amr test_launch_sim.launch.py 
```

Initiate voice/speech
```
python3 /home/jetson/agv/src/amr/venv_run/kokoro/kokoro_launcher.py
```

Serial connection
```
ros2 run serial_test serial_test
```

Mapping process
```
ros2 launch launch_amr map.launch.py 
```

Independant navigation process for testing without custom plugin
```
ros2 launch launch_amr nav.launch.py 
```

Navigation process with custom plugin
```
ros2 launch launch_amr nav_yolo.launch.py 
```

[Launch bringup and navigation without plugin](https://github.com/LeoAquinas/amr/blob/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/launch/launch/demo_launch.launch.py)
```
ros2 launch launch_amr launch/launch/demo_launch.launch.py
```

[Launch bringup and navigation with plugin](https://github.com/LeoAquinas/amr/blob/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/launch/launch/demo_launch_without_voice.launch.py)
```
launch/launch/demo_launch_without_voice.launch.py
```



## Extra Scripts
**The purpose of these scripts were for evaluation of FYP data and has no direct effect on the functions of the AMR**
1. Bag recording and simple commander initiation [here](launch/launch/start_nav.launch.py)
2. To get SLAM results from OpenLORIS dataset for comparison of SLAM methodology with other existing research [here](https://github.com/LeoAquinas/amr/blob/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/launch/launch/compare_mapping.launch.py)
3. Need to run [this](https://github.com/LeoAquinas/amr/blob/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/launch/launch/comparison_static_transforms.launch.py) to provide transform for OpenLORIS dataset before SLAM so that transform can be obtained
4. [This](https://github.com/LeoAquinas/amr/tree/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/odom_rotator) is to rotate transfrom of RTABMAP such that the transfrom is in correct frame during movement comparison
5. [This](https://github.com/LeoAquinas/amr/tree/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/imu_fusion) is to merge IMU data from OpenLORIS dataset for SLAM algorithms
6. [This](https://github.com/LeoAquinas/amr/tree/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/camera_info) is to get camera info from OpenLORIS dataset for evaluation
7. [This](https://github.com/LeoAquinas/amr/tree/7a5d45ad16c2ddb3c9360e3bd72d557752553d93/voice_control) are all scripts used for development of voice scripts, can ignore but is good for reference


### Deprecated
Start ORBSLAM3
```
ros2 run orbslam3 stereo /home/jetson/agv/src/vslam/orbslam3_ros2/vocabulary/ORBvoc.txt /home/jetson/agv/src/vslam/orbslam3_ros2/config/stereo/RealSense_D435i.yaml false
```

Nav2 without plugin with AMCL
```
ros2 launch launch_amr launch/launch/nav_simple_commander.launch.py
```

Nav2 with plugin with AMCL
```
ros2 launch launch_amr launch/launch/nav_yolo.launch.py
```
