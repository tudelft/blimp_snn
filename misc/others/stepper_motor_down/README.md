### What is this repository for? ###

* A simple C++ class created to provide an easily interaction with the geared stepper motor 28BYJ48 through the ULN2003APG driver. 
* Version: 0.1

### What do I need to compile/run the code? ###

* A [Raspberry Pi](https://www.raspberrypi.org) with a Linux distro installed (i.e. Raspbian)
* a C/C++ compiler (gcc-g++)
* the [WiringPi library](http://wiringpi.com)
* cmake
* git

### Setup ###

#### How to download and install the WiringPi library ####
[Here](http://wiringpi.com/download-and-install) you can find all the instructions.    


#### Download, compile and run the code in this repo ####

At this time there are two main branches: master and testing.
The first one corresponds to the 'stable' version, the other to the 'testing' version, as you can easily guess.

Download the 'stable' version:

    git clone https://crish4cks@bitbucket.org/crish4cks/steppermotor.git

Download the 'current' version:

    git clone -b testing https://crish4cks@bitbucket.org/crish4cks/steppermotor.git

Build the sources:
    
    cd steppermotor/
    mkdir build && cd build
    cmake ..
    make

NOTE: The whole code has been successfully tested on Raspberry Pi 1 model B, [board revision 000f](http://elinux.org/RPi_HardwareHistory) and on Raspberry Pi 3 model B, [board revision a22082](http://elinux.org/RPi_HardwareHistory)

Perform a simple test:

    ./Test_1


If you want to keep clean the build directory, you can use this simple bash script:
    
    # Clean the 'build' directory
    # 
    #!/bin/bash
    rm -rf CMakeFiles \
           CMakeCache.txt \
           cmake_install.cmake \
           libStepperMotor.a \
           Makefile \
           Test*

### Who do I talk to? ###

* Repo owner/admin
