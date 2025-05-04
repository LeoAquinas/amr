# README
This is a repository for a final year project pertaining to the HRI system of an AMR with the functionalities of:
  1. Autonomous Navigation
  2. Voice/Speech Control
  3. Teleoperation Control
  4. Touchpad Inputs

## Packages
| Packages | Usage |
|---|---|
|`amr`|Main Robot Package|
|`rtab-map`|Robot Mapping|
|`ORBSLAM3`|To Provide Visual Odometry|
|`sllidar`|Package for LiDAR Functionality|
|``|Package for IntelRealsense Camera|
|`robot_localization`|Package for Unscented Kalman Filter|


## Usage
Clone repository:
```
git clone https://github.com/LeoAquinas/amr.git
```
Setup Dependencies
### RTAB-Map
Clone from [source](https://github.com/introlab/rtabmap_ros/tree/ros2)



### ORBSLAM3
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


### robot_localization
Clone package:
```
git clone https://github.com/cra-ros-pkg/robot_localization
```

### sllidar_ros2
[Package](https://github.com/Slamtec/sllidar_ros2) for LiDAR
```
git clone https://github.com/Slamtec/sllidar_ros2.git
```

### IntelRealSense

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

###



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

## Testing connection of FS-i6X controller
Receiver connected to UART pins on Jetson. To test if ports are available, run:
```
ls /dev/ttyTHS*
```
User should see something like this:
```
/dev/ttyTHS1 /dev/ttyTHS2
```
Once the receiver has been connected to the Jetson, the IBUS data stream can be checked with the screen package.
```
sudo apt install screen
screen /dev/ttyTHS1 115200  # Exit with Ctrl+A → K
```


## Commands
Launch robot bringup to start robot
```
ros2 launch
```
Launch voice control
```
```
Launch RTAB-Map for mapping
```
```
Launch Nav2
```
```




