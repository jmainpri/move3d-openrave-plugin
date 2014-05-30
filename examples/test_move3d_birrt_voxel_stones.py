#!/usr/bin/env python
# Jim Mainprice, ARC
# March 2013
# Worcester Polytechnic Institute

# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.
# http://openrave.org/docs/latest_stable/command_line_tools/

from openravepy import *
from numpy import *
import pdb
import sys
import os

from test_move3d_birrt_stones import *


if __name__ == "__main__":

    print "START OPENRAVE"
    planner = TwoDPlanner()
    planner.SetCamera()

    collisionChecker = RaveCreateCollisionChecker( planner.env,'VoxelColChecker')
    collisionChecker.SendCommand('SetDimension extent 500 800 30 voxelsize 5')
    collisionChecker.SendCommand('Initialize')
    collisionChecker.SendCommand('SetDrawing on')
    planner.env.SetCollisionChecker( collisionChecker ) 

    while True :
        
        planner.run()

        trajectory = RaveCreateTrajectory(planner.env, "")
        trajectory.deserialize(open("traj_0.txt", 'r').read())

        planner.robot.GetController().SetPath(trajectory)
        planner.robot.WaitForController(0)

        print "Press return replan."
        sys.stdin.readline()

