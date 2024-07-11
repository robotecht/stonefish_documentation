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

To install the updated version of the stonefish follow the instructions given in the repository:

[**Stonefish - Heriot-Watt Repository**](https://github.com/oceansystemslab/HeriotWattStonefishSim)

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

To install stonefish Library follow the instructions at: https://stonefish.readthedocs.io/

**Dependencies**
- Install [OpenGL Mathematics](https://github.com/g-truc/glm) (libglm-dev, version >= 0.9.9.0)
  ```bash
  sudo apt install libglm-dev
  ```
- Install [SDL2](https://github.com/libsdl-org/SDL)(libsdl2-dev)
  ```bash
  sudo apt install libsdl2-dev
  ```







