# Stonefish Simulator:
Stonefish is a powerful C++ library designed specifically for simulating and visualizing marine robots. It excels at rendering realistic underwater environments and the interactions between robots and their surroundings.  This makes Stonefish ideal for researchers and developers working on aquatic robotics projects.  This tutorial will guide you through using Stonefish to create real-time, 3D visualizations of your robot data.

## System Requirements

Stonefish requires a modern computer for smooth operation. Here's a breakdown of the minimum specifications:

- **Processor:** Multi-core processor (recommended for real-time performance)
- **Graphics Card:** Nvidia GeForce 400 series or AMD Radeon HD 5000 series (or equivalent with OpenGL 4.3 support)
- **Operating System:** Linux Ubuntu 22.04.4 LTS - Installation Guide: [Ubuntu Installation](https://www.youtube.com/watch?v=oNEwEQ0uU1Y)

**Note:** While a powerful GPU is recommended for full functionality, Stonefish can run basic simulations in console mode with a less capable graphics card.

## ROS2
ROS 2 (Robot Operating System 2) is the next-generation framework for building robot applications. It empowers users and developers to leverage a rich ecosystem of libraries and tools, streamlining the development process.

This tutorial guides you through implementing the Stonefish application using ROS 2. Stonefish is particularly well-suited for aquatic environments, and we'll explore its functionalities in this context.

Before diving in, let's install ROS 2 - Humble Hawksbill. Here are some helpful resources:

- [**Official ROS 2 Documentation:**](https://docs.ros.org/en/humble/index.html) This comprehensive resource provides detailed installation instructions for various platforms.
- [**ROS 2 Tutorial (Video):**](https://www.youtube.com/watch?v=0aPbWsyENA8&list=PLLSegLrePWgJudpPUof4-nVFHGkB62Izy) This video tutorial offers a visual guide to installing ROS 2 - Humble Hawksbill.

**Note:** Use the information in the video as a general guideline for installation, follow the ROS2 documentation as the main guide.

## Stonefish Simulator Installation

The updated stonefish simulator is available in the following repository:

[**Stonefish - Heriot-Watt Repository**](https://github.com/oceansystemslab/HeriotWattStonefishSim)

Follow the instructions below to install the simulator in your linux system.

### NVIDIA Settings
**Nvidia Performance Mode (for Native Ubuntu version)**
Install nvidia-prime
```bash
sudo apt install nvidia-prime
```

And then set nvidia in performance mode:
```bash
sudo prime-select nvidia
```

### Installation

The instructions are adapted from the original stonefish repository: https://stonefish.readthedocs.io/

**Dependencies**
- Install [OpenGL Mathematics](https://github.com/g-truc/glm) (libglm-dev, version >= 0.9.9.0)
  ```bash
  sudo apt install libglm-dev
  ```
- Install [SDL2](https://github.com/libsdl-org/SDL)(libsdl2-dev)
  ```bash
  sudo apt install libsdl2-dev
  ```
> **NOTE:** SDL2 library may need a small fix to the CMake configuration file, to avoid build errors. Remove a space after -lSDL2 in /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake.
  ```bash
  sudo xdg-open /usr/lib/x86_64-linux-gnu/cmake/SDL2/sdl2-config.cmake
  ```
 > This will open the sdl2-config.cmake file. Press **Ctrl** + **H** to open the replacement window, and replace "lSDL2 " with "lSDL2", then select replace all option.

- Install [Freetype](https://freetype.org/)(libfreetype6-dev)
  ```bash
  sudo apt install libfreetype6-dev
  ```

**Stonefish Installation**

Create a workspace

```bash
mkdir -p stonefish_ros2_ws/src
```
Clone the Heriot-Watt Stonefish simulator repository Or use the package shared with you
```bash
git clone git@github.com:oceansystemslab/HeriotWattStonefishSim.git
```
Copy the cola2_msgs_ros2,cola2_stonefish_ros2 and stonefish_ros2 into stonefish_ws/src folder

Navigate to stonefish directory and build this following the directions in [Stonefish Repository](https://github.com/patrykcieslak/stonefish)

Build packages in the stonefish_ws workspace

```bash
cd ~/stonefish_ws
colcon build
```
Source the installation again
```bash
source install/setup.bash
```
Now Stonefish ros2 installation is completed
Launch the simulator environment using the available launch files in the library
```bash
ros2 launch cola2_stonefish bluerov_fls_simulation.launch.py
```

![image](https://github.com/user-attachments/assets/1cf0c767-9942-4081-8d2e-2932d2235f08)

Bluevolta
```bash
ros2 launch cola2_stonefish bluevolta.launch.py
```
Essence
```bash
ros2 launch cola2_stonefish essence.launch.py
```
------------------------------------------------------------------------------------------------------------------------------------------------

## WSL2 Version
In order to use stonefish within wsl2 you should:
- put gpu in nvidia performance mode in the nvidia control panel in the global parameters and  set opengl to use the nvidia gpu
- then launch wsl and write: 
```bash
sudo add-apt-repository ppa:kisak/kisak-mesa
sudo apt-get dist-upgrade
sudo apt install mesa-utils
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
sudo apt install freeglut3-dev
```
Finally use glxgears and the performance monitor in windows to be sure that it is using the GPU

## Installation

**Stonefish Installation**

Create a workspace

```bash
mkdir -p stonefish_ros2_ws/src
```
Clone the Heriot-Watt Stonefish simulator repository Or use the package shared with you
```bash
git clone git@github.com:oceansystemslab/HeriotWattStonefishSim.git
```
Copy the cola2_msgs_ros2,cola2_stonefish_ros2 and stonefish_ros2 into stonefish_ws/src folder

Navigate to stonefish directory and build this following the directions in [Stonefish Repository](https://github.com/patrykcieslak/stonefish)

Build packages in the stonefish_ws workspace

```bash
cd ~/stonefish_ws
colcon build
```
Source the installation again
```bash
source install/setup.bash
```
Now Stonefish ros2 installation is completed
Launch the simulator environment using the available launch files in the library
```bash
ros2 launch cola2_stonefish bluerov_fls_simulation.launch.py
```
Then put stonefish_ros, cola2_msgs and cola2_stonefish in your catkin workspace's src folder. Install pybind11 via: 
```bash
apt install python3-pybind11
```
Finally build stonefish_ros and cola2_stonefish
#### Nvidia Performance Mode (for Native Ubuntu version)
Install nvidia-prime 
```bash
apt install nvidia-prime
```
And then set nvidia in performance mode:
```bash
prime-select nvidia
```
## Instructions
To launch bluerov sim :
```bash
ros2 launch cola2_stonefish bluerov_simulation.launch 
```
To launch blueboat and bluerov sim :
```bash
ros2 launch cola2_stonefish asv_usv_sim.launch 
```
To move the bluerov use thrusters:
```bash
ros2 topic pub /bluerov/controller/thruster_setpoints_sim std_msgs/Float64MultiArray '{data:[0.01, 0.01, 0.01, 0.01, 0.4, 0.4]}'
```
To move the blueboat use thrusters:
```bash
ros2 topic pub /blueboat/controller/thruster_setpoints_sim std_msgs/Float64MultiArray '{data:[0.01, 0.01]}'
```
Thrusters order is the same as in the scenario file

## Sensors
Stonefish comes with many sensors as explained in : https://stonefish.readthedocs.io/en/latest/sensors.html
You can access the different topics and check them. You just need to define them as written in the above documentation and add in the scenario file a rospublisher element to the sensor as:
```xml
<sensor name="gps" type="gps" rate="1.0">
        <link name="Vehicle"/>
        <origin rpy="0.0 0.0 0.0" xyz="0 0 -0.74"/>
        <noise ned_position="0.5"/>
        <ros_publisher topic="/bluerov/navigator/gps"/>
</sensor>
```
## Examples : BlueROV
To go down:
```bash
ros2 topic pub /bluerov/controller/thruster_setpoints_sim std_msgs/Float64MultiArray '{data:[0.0, 0.0, 0.0, 0.0, 0.7, 0.7]}'
```
Stay at a stable depth:
```bash
ros2 topic pub /bluerov/controller/thruster_setpoints_sim std_msgs/Float64MultiArray '{data:[0.0, 0.0, 0.0, 0.0, 0.4, 0.4]}'
```
## Examples : BlueROV Stereo
To use the StereoCamera config, launch
```bash
ros2 launch cola2_stonefish bluerov_stereo_simulation.launch
```
## Examples : BlueROV FLS
To use the FLS config, launch
```bash
ros2 launch cola2_stonefish bluerov_fls_sim.launch
```

## Instructions OpenGym
To launch bluerov opengym :
```bash
ros2 launch cola2_stonefish bluerov_simulation_opengym.launch 
```
