### aromi_ros
-------------------------
aromi_ros is a ROS package for RICAL quadrotors.


## Install ROS
Ros installation [tutorials](http://wiki.ros.org/kinetic/Installation/Ubuntu)


## Catkin workspace
http://wiki.ros.org/catkin/Tutorials/create_a_workspace


## Intel RealSense
```
# Update system
sudo apt update			
sudo apt upgrade -y		

# Install dependencies
sudo apt-get install ros-kinetic-ddynamic_reconfigure
sudo apt install xorg-dev libglu1-mesa-dev
sudo apt install git libssl-dev libusb-1.0-0-dev pkg-config -y		
sudo apt install cmake python3-dev 

# Clone the repository under home
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense

# Install udev rules
sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && udevadm trigger

# Create the destination directory
mkdir build && cd build

# Remove extra files if this is not your first run
xarg sudo rm < install_manifest.txt
rm CMakeCache.txt

# Compile librealsense
export CC=/usr/bin/gcc-7
export CXX=/usr/bin/g++-7
cmake -D CMAKE_BUILD_TYPE="Release"\
-D FORCE_LIBUVC=ON \
-D BUILD_PYTHON_BINDINGS=ON \
-D BUILD_EXAMPLES=ON ..
make -j4
sudo make install
sudo ldconfig
sudo reboot

# clone realsense-ros repository and compile it.
cd catkin_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git
cd realsense-ros/
git checkout `git tag | sort -V | grep -P "^\d+\.\d+\.\d+" | tail -1`
cd ..
catkin_init_workspace
cd ..
catkin_make clean
catkin_make -DCATKIN_ENABLE_TESTING=False -DCMAKE_BUILD_TYPE=Release
catkin_make install

#  try to test camera node in ROS.
roslaunch realsense2_camera rs_camera.launch
```

## Install PX4 Firmware
```
# Clone the PX4 Firmware and all its submodules
cd ~
git clone https://github.com/PX4/Firmware.git
cd ~/Firmware
git submodule update --init --recursive

# Install PX4 dependencies
sudo apt-get update -y
sudo apt-get install git zip qtcreator cmake \
    build-essential genromfs ninja-build exiftool -y

# Install xxd (package depends on version)
which xxd || sudo apt install xxd -y || sudo apt-get install vim-common --no-install-recommends -y

# Required python packages
sudo apt-get install python-argparse \
    python-empy python-toml python-numpy python-yaml \
    python-dev python-pip -y
sudo -H pip install --upgrade pip 
sudo -H pip install pandas jinja2 pyserial cerberus

# optional python tools
sudo -H pip install pyulog

# ninja build system
sudo apt-get install ninja-build -y

# jMAVSim simulator
sudo apt-get install ant openjdk-8-jdk openjdk-8-jre -y

# gazebo dependencies
sudo apt-get install protobuf-compiler libeigen3-dev libopencv-dev -y
sudo apt-get install python-wstool python-rosinstall-generator python-catkin-tools -y

# Dependencies for NuttX based hardware (Pixhawk) 
sudo apt-get install python-serial openocd \
    flex bison libncurses5-dev autoconf texinfo \
    libftdi-dev libtool zlib1g-dev -y

# Remove any old versions of the arm-none-eabi toolchain.
sudo apt-get remove gcc-arm-none-eabi gdb-arm-none-eabi binutils-arm-none-eabi gcc-arm-embedded
sudo add-apt-repository --remove ppa:team-gcc-arm-embedded/ppa

# Execute the script below to install GCC 7-2017-q4
pushd .
cd ~
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/7-2017q4/gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
tar -jxf gcc-arm-none-eabi-7-2017-q4-major-linux.tar.bz2
exportline="export PATH=$HOME/gcc-arm-none-eabi-7-2017-q4-major/bin:\$PATH"
if grep -Fxq "$exportline" ~/.profile; then echo nothing to do ; else echo $exportline >> ~/.profile; fi
popd

# Restart the machine and check the version
arm-none-eabi-gcc --version

# Output should be ...
arm-none-eabi-gcc (GNU Tools for Arm Embedded Processors 7-2017-q4-major) 7.2.1 20170904 (release) [ARM/embedded-7-branch revision 255204]
Copyright (C) 2017 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

# Setup for .bashrc
cd ~
nano .bashrc

# Add
export QT_X11_NO_MITSHM=1
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:~/Firmware
export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:~/catkin_ws/src/avoidance/avoidance/sim/models:~/catkin_ws/src/avoidance/avoidance/sim/worlds

# Build and run simulation
make px4_sitl_default gazebo
```


## Install MAVROS and PX4 avoidance package
```
sudo apt install python-catkin-tools
sudo apt install ros-kinetic-mavros ros-kinetic-mavros-extras
sudo apt install ros-kinetic-stereo-image-proc
sudo apt install ros-kinetic-image-view
sudo apt-get install libgstreamer1.0-*
sudo apt-get install ros-kinetic-rqt-reconfigure
sudo apt install rosbash

# Install the geographiclib dataset
wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
chmod +x install_geographiclib_datasets.sh
sudo ./install_geographiclib_datasets.sh

# Install avoidance module dependencies (pointcloud library and octomap).
sudo apt install libpcl1 ros-kinetic-octomap-* ros-kinetic-yaml-*

# Clone this repository in your catkin workspace in order to build the avoidance node
cd ~/catkin_ws/src
git clone https://github.com/PX4/avoidance.git
catkin_make (catkin_make -DCMAKE_BUILD_TYPE=Release)

# Simulate a forward looking stereo camera running OpenCV's block matching algorithm
roslaunch local_planner local_planner_stereo.launch

# The disparity map from stereo-image-proc is published as a stereo_msgs/DisparityImage message, 
# which is not supported by rviz or rqt. To visualize the message, either run:
rosrun image_view stereo_view stereo:=/stereo image:=image_rect_color

# or publish the DisparityImage as a simple sensor_msgs/Image
rosrun topic_tools transform /stereo/disparity /stereo/disparity_image sensor_msgs/Image 'm.image'

# Simulate a forward looking kinect depth sensor
roslaunch local_planner local_planner_depth-camera.launch

# Simulate three kinect depth sensors
roslaunch local_planner local_planner_sitl_3cam.launch

# Start the global_planner and use it for avoidance in offboard mode.
roslaunch global_planner global_planner_stereo.launch

# From the command line, you can also make Gazebo follow the drone, if you want.
gz camera --camera-name=gzclient_camera --follow=iris

# You will see the Iris drone unarmed in the Gazebo world. 
# To start flying, there are two options: OFFBOARD or MISSION mode. 
# For OFFBOARD, run: In another terminal
rosrun mavros mavsys mode -c OFFBOARD
rosrun mavros mavsafety arm

# Then the drone will start moving towards the goal. 
# The default x, y goal position can be changed in Rviz by clicking on the 2D Nav Goal button and then choosing the new goal x and y position by clicking on the visualized gray space. 
# If the goal has been set correctly, a yellow sphere will appear where you have clicked in the grey world. 

# One can plan a new path by setting a new goal with the 2D Nav Goal button in rviz. 
# The planned path should show up in rviz and the drone should follow the path, updating it when obstacles are detected. 
# It is also possible to set a goal without using the obstacle avoidance (i.e. the drone will go straight to this goal and potentially collide with obstacles). 
# To do so, set the position with the 2D Pose Estimate button in rviz.

# Start the safe_landing_planner and use it to land safely in mission or auto land mode. To run the node:
roslaunch safe_landing_planner safe_landing_planner.launch

# You will see an unarmed vehicle on the ground. Open QGroundControl, either plan a mission with the last item of type Land or fly around the world in Position Control, click the Land button on the left side where you wish to land. At the land position, the vehicle will start to descend towards the ground until it is at loiter_height from the ground/obstacle. Then it will start loitering to evaluate the ground underneeth. If the ground is flat, the vehicle will continue landing. Otherwise it will evaluate the close by terrain in a squared spiral pattern until it finds a good enough ground to land on.


## Install and build aromi_ros
```
cd ~/catkin_ws/src
git clone https://github.com/kimfeel/aromi_ros.git
catkin_make
```


## Run on Hardware
Reference: https://github.com/PX4/avoidance

