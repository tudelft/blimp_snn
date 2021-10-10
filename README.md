# blimp_snn

### *Evolved neuromorphic radar-based altitude controller for an autonomous open-source blimp* **official repository**

* Supporting paper: [https://arxiv.org/abs/2110.00646](https://arxiv.org/abs/2110.00646)

In this work, we propose an evolved altitude controller based on a SNN for a robotic airship which relies solely on the sensory feedback 
provided by an airborne radar. Starting from the design of a lightweight, low-cost, open-source airship, we also present a SNN-based 
controller architecture, an evolutionary framework for training the network in a simulated environment, and a control scheme for 
ameliorating the gap with reality. The system's performance is evaluated through real-world experiments, demonstrating the advantages 
of our approach by comparing it with an artificial neural network and a linear controller. The results show an accurate tracking of the 
altitude command with an efficient control effort.

![gondola](media/intro.png)

### Repository Contents

This repository contains the following materials:
* Assembly guide and bill of materials
* 3D printing files of the airship platform
* ROS interface for blimp teleoperation
* ROS interface for blimp autonomous control: PID, ANN, SNN
* Evolutionary framework for ANN and SNN training in simulation
* Radar signal processing algorithm integrated in ROS (*to be added*)

### Installation

* On the **computer**:
  1. Configure repo:
     ```shell
     cd setup
     chmod +x install_basics.sh
     sudo ./install_basics.sh
     ```
     Make sure after this that you have the same folder configuration for *catkin_ws* as the one shown [here](#content-and-folder-structure)
  2. Create a conda environment with Python 2.7:
     ```shell
     conda create -n myenv python=2.7
     ```
  3. Finally, install [*PyTorch 1.4.0*](https://varhowto.com/install-pytorch-1-4-0/), [*OpenCV*](https://linuxize.com/post/how-to-install-opencv-on-ubuntu-18-04/), [*PySNN*](https://github.com/BasBuller/PySNN), [*DEAP*](https://deap.readthedocs.io/en/master/installation.html), *matplotlib*, and more, as needed.


* On the **Raspberry Pi**:
  1. Install CMake, OpenCV 4.1.2 and ROS Kinetic:
     ```shell
     cd setup
     chmod +x cmake_installation.sh
     sudo ./cmake_installation.sh
     chmod +x opencv_installation.sh
     sudo ./opencv_installation.sh
     chmod +x ros_kinetic_install.sh
     sudo ros_kinetic_install.sh
     ```
  2. Configure repo:
     ```shell
     cd setup
     chmod +x radar_mine_install.sh
     sudo ./radar_mine_install.sh
     ```
  4. Install [PyTorch](https://gist.github.com/akaanirban/621e63237e63bb169126b537d7a1d979), *matplotlib*, and more, as needed.

### Basic Usage

* To teleoperate the airship from the computer keyboard:
  1. Follow the instructions in [here](/read_me/rpi_ssh_connection/README.md) to connect to the Raspberry Pi via SHH.
  2. Follow the instructions in [here](/read_me/ros_multiple_machines/README.md) to launch ROS both from the ground station computer and airborne RPi.
  3. Compile and source the ROS workspace
     ```shell
     cd blimp_snn/ros_radar_mine/catkin_ws
     catkin build
     source devel/setup.bash
     ```
  4. Launch the keyboard teleoperation node and motor control nodes:
     ```shell
     roslaunch motor_control motor_control.launch
     ```

### Content and Folder Structure

In this section, a detailed view of the final folder structure and function of the available ROs packages + evolutionary
framework to train ANN and SNN controllers in simulation is delineated.

```
.
├── media                                   # Media for this repo
├── misc                                    # Irrelevant now: some first tests to move the actuators
│
├── read_me                                 # Important READMEs!
│   ├── radar_wp                                 # Radar processing algorithm: full explanation
│   ├── ros_cheat_sheet                          # ROS cheatsheet
│   ├── ros_multiple_machines                    # Launching ROS both from computer and Raspberry Pi (RPi)
│   └── rpi_ssh_connection                       # Connecting to RPi through SSH
│
├── ros_radar_mine                          # Where EVERYTHING is DONE
│   │
│   ├── catkin_ws                           # ROS MAIN WORKSPACE !!!
│   │   └── src
│   │       ├── catkin_simple                    # Package to simplify ROS builds (downloaded)
│   │       ├── mocap_optitrack                  # Package to record from the OptiTrack Motion Capture System
│   │       ├── motor_control                    # Package to control the airship (PID,ANN,SNN) and actuators
│   │       ├── plotjuggler_msgs                 # Package for real-time plotting (downloaded)
│   │       └── plotjuggler-ros-plugins          # Package for real-time plotting (downloaded)
│   │
│   ├── neuro_learning                       # EVOLUTIONARY ENVIRONMENT TO TRAIN ANNs and SNNs !!!
│   │   └── controller
│   │       ├── blimp_model                      # Folder containing mathematical model of blimp
│   │       ├── config                           # Config YAML files to set configuration for training
│   │       ├── evol_algo                        # Evolutionary algorithms used
│   │       ├── evol_funcs                       # Evolutionary functions (for initialization, mutation, etc.)
│   │       ├── evol_main                        # Evolutionary main file
│   │       ├── extra                            # Additional funcitons for plotting, etc.
│   │       ├── network                          # Definition of several types of ANN and SNN networks
│   │       ├── pid                              # Definition of PID controller
│   │       ├── save                             # Where the trained networks are saved
│   │       └── testing                          # For plotting and testing
│   │
│   ├── radar_filters                       # Irrelevant now: some files to test several radar MA filters
│   │
│   └── system_ident                        # Blimp model identification with Matlab
│
├── setup                                   # SETUP / INSTALLATION FILES !!!
│   ├── cmake_installation.sh                           # CMake installation on RPi
│   ├── opencv_installation.sh                          # OopenCV installation on RPi
│   ├── radar_mine_install_basics.sh
│   ├── install_basics.sh                               # MAIN INSTALLATION FILE: EXECUTE FOR PROPER REPO CONFIG!
│   ├── radar_position2go_install.sh
│   ├── ros_kinetic_install.sh                          # ROS Kinetic installation on RPi
│   └── wpa_supplicant.conf                             # To setup default Wi-Fi connections for the RPi
│
└── SolidWorks.zip                          # SolidWorks and 3D printing files to replicate the gondola


```

### Summary

The gondola can be 3D printed and is assembled in a modular fashion. Its open configuration makes it
notably versatile, leaving room for the selection of custom actuators and sensors, which can be easily
interchanged. Moreover, it merely uses one low-power micro servomotor and two coreless DC motors
to achieve the three main motion primitives: forward, upward/downward, and yaw motion. The DC
motors are accommodated on both ends of the rotary shaft, which is controlled by the servo. The
rotation starts at 0º (upward movement) and can go up to 180º (downward movement). The thrust of
each of the propellers can be controlled independently, to achieve the desired yaw.
![gondola](media/gif_blender.gif)

The following image provides a detailed view of the whole assembly schematics for easy replication:

![gondola](media/circuit.png)

| Component  | Price [$] |  | Component | Price [$] |
| ------------- | ------------- | --- | ------------- | ------------- |
| Raspberry Pi W Zero  | 10  | | 8520 Coreless Motors | 8.00 |
| DRV8833 Motor Driver  | 4.95  | | PowerBoost 500 Basic | 9.95 |
| Sub-micro Servo SG51R | 5.95 | | 550mA 3.8V Li-Po Battery | 7.95 |


For more technical details on the electronic components, their properties and interconnection, please visit
the following link to [@marina-go-al](https://github.com/marina-go-al) 's master's thesis: [*link to be added here*]

### Citation
```
@misc{gonzalezalvarez2021evolved,
      title={Evolved neuromorphic radar-based altitude controller for an autonomous open-source blimp}, 
      author={Marina González-Álvarez and Julien Dupeyroux and Federico Corradi and Guido de Croon},
      year={2021},
      eprint={2110.00646},
      archivePrefix={arXiv},
      primaryClass={cs.RO}
}
```

### Contact

GitHub: [@marina-go-al](https://github.com/marina-go-al), [@JuSquare](https://github.com/JuSquare)

LinkedIn: [@mgonzalezalvar](https://www.linkedin.com/in/mgonzalezalvar/), [@julien-dupeyroux](https://www.linkedin.com/in/julien-dupeyroux-12a66a8a/)