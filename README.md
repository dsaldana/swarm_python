# An interface to control the Swarm Simulator in Python

Main instructions about how to build the project in [https://bitbucket.org/osrf/swarm/wiki/Tutorial_1-How_to_create_your_Swarm_controller]:

## Requirements

Download and Install the Swarm simulator from:
https://bitbucket.org/osrf/swarm/wiki/Install.md

Before compiling, open the file **RobotPlugin.hh** and replace the keyword
**protected** for **public**.

## Install the C++ bridge for Python
Get this project

    git clone https://github.com/dsaldana/swarm_python.git
    cd swarm_python

Create a build directory:

    mkdir build
    cd build

Build your plugin (controller):

    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4

Install your plugin:

    sudo make install
    

## Run the swarm simulator
Run an example in the same folder where the script **controller.py** is located:
    
    cd ..
    gazebo --verbose worlds/ground_simple_2.world






~/swarmsu/swarm_python/circular_example