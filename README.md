# A c++ interface to control swarms in python

Main instructions about how to build the project in [https://bitbucket.org/osrf/swarm/wiki/Tutorial_1-How_to_create_your_Swarm_controller]:



Create a build directory:

    mkdir build
    cd build

Build your plugin (controller):

    cmake .. -DCMAKE_INSTALL_PREFIX=/usr
    make -j4

Install your plugin:

    sudo make install
    

Run an example:
    
    gazebo --verbose worlds/ground_simple_2.world

# Error
We want to create a PyCFunction with a non-static method. 
The current problem is described here:
[http://stackoverflow.com/questions/22869709/how-do-i-pass-a-instance-member-function-as-a-pycfunction-type-using-the-python]


