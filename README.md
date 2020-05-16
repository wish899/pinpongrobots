# Ping Pong Robot Simulation for ECE 470 - Spring 2020

## Objective
The goal of this project is to have two robots (UR3) simulate rallying a ping-pong ball back and forth through the use of simple hits. The robots will detect the ping pong ball and calculate their position based on the ball's location and make predicative measurements to get ready to hit the ball back.

## Execute Program
We are connecting remotely to coppelia through the python script, so to run the simulation, run project.ttt file and then run the python script stest.py

## Description of files
stest.py is the main file for demonstrating our use of our simulator (CoppeliaSim). This file is the basis for demonstrating basic movement of the robot and the basis for collecting vision sensor data. 

inverse_kinematics.py contains our code for computing the joint angles of the robot using our customized elbow-up configuration given the desired world coordinates

bouncing_ball.py contains our code for finding the linear trajectory of the ball.

sim.py contains all of our function definitions for use with the robot. 

simConst.py contains all of our constant variables for use with the robot. This file groups all of them together into specific subclasses which makes it easier to find the variables.

simpleTest.py is our old code for interacting with the simulator. It contains the code we used to generate the videos for the project updates. 

ur3_orig.lua is the original code on coppelia for the UR3.

ur3_remote.lua is the most recent up to date code on coppelia for the UR3.

### Other notes
The python script is written in python3 and requires numpy, scipy and the modern_robotics package. It also uses the simx API, which is used to connect remotely to the CoppeliaSim scene

## Forward Kinematics
We implement our forward kinematics in simpleTest.py. In it, we convert the euler angles which we get from the simulation into the Rotation Matrix required for the model using the `scipy.spatial.transform.rotation` module. Using this and the position, we then get the original transformation matrix M. 

By inspection, we got the axes of rotation (w) for our joints and got the location of these joints (the qs ) through the remote API. We then calculated the cross product using `np.cross()` to get our linear velocities (v). Using w and v, we create the set of screw axes, S. 

From there, we use the `mr.FKinSpace()` subroutine to calculate the new transformation matrix for a certain set of joint angles. To verify our answer visually, we move a dummy object to the new calculated frame of the end-effector after setting the new joint angles. 

## Inverse Kinematics

We implement our inverse kinematics in inverse_kinematics.py and invoke it. We use a customized elbow-up configuration of the UR3 to generate the joint positions. 

## Trajectory Generation

We implement our line trajectory generation in bouncing_ball.py. In it, we take the last recorded position and velocity of the ball as it left the wall and predict the position of the ball when it reaches a position close to the robot. Here, we just use a linear approximation for the equation. 

## Other files that we did not end up using in the project
project_vish.ttt contains the file that Vishal used for initial simulation testing. 
