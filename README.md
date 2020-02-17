Code for Ping Pong Robot Simulation for ECE 470 - Spring 2020
The goal of this project is to have two robots (UR3) simulate rallying a ping-pong ball back and forth through the use of simple hits. The robots will detect the ping pong ball and calculate their position based on the ball's location and make predicative measurements to get ready to hit the ball back.

We are connecting remotely to coppelia through the python script, s
o just open the .ttt file and run it.

simpleTest.py is the main file for demonstrating our use of our simulator (CoppeliaSim). This file is the basis for demonstrating basic movement of the robot and the basis for collecting vision sensor data. 

sim.py contains all of our function definitions for use with the robot. 

simConst.py contains all of our constant variables for use with the robot. This file groups all of them together into specific subclasses which makes it easier to find the variables.

ur3_orig.lua is the original code on coppelia for the UR3.

ur3_remote.lua is the most recent up to date code on coppelia for the UR3.
