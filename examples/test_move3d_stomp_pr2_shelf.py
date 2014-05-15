#!/usr/bin/env python
# Copyright (c) 2014 Worcester Polytcchnic Institute
#   Author: Jim Mainprice <jmainprice@wpi.edu>

# -*- coding: utf-8 -*-

from openravepy import *
from numpy import *
from misc_transform import *

import time
import sys

if __name__ == "__main__":
      
    # load the environment if it is not already loaded
    try:
        orEnv
    except NameError:
        orEnv = Environment()
        orEnv.SetViewer('qtcoin')
    
    # load the robot into the environment
    orEnv.Reset()
    orEnv.Load('robots/pr2-beta-static.zae')
    orEnv.Load('data/shelf.kinbody.xml')

    T_cam = array(  [[ 0.81240217, -0.20717684,  0.54505089, -1.11184239], \
                     [-0.58198477, -0.34581952,  0.73600448, -2.30210519], \
                     [ 0.03600615, -0.91514295, -0.40151829,  2.17947888], \
                     [ 0.,          0.,          0.,          1.        ]] )

    orEnv.GetViewer().SetCamera(T_cam)

    T = eye(4)
    T[0,3] = 0.0
    T[1,3] = 0.0
    T[2,3] = 0.0

    for b in orEnv.GetBodies():
        print "name : ", b.GetName()
        if b.GetName() == "Shelf":
            shelf = b

    # Get body
    shelf.SetTransform( array( MakeTransform( rodrigues([-pi/2,0,0]), matrix([0.7,-0.5,0]) ) ) )
    print array(MakeTransform( rodrigues([-pi/2,0,0]), matrix([0.7,-0.5,0]) ) )

    # Get robot
    robot = orEnv.GetRobots()[0]
    robot.SetTransform(T)

    # ---------------------------------------------------------------
    # Setup voxel collision checker
    indices_vox = [27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37]
    robot.SetActiveDOFs(indices_vox)

    # Print the limits and get active links
    links = []
    for jIdx, j in enumerate(robot.GetJoints()):
        if jIdx not in indices_vox :
            continue
        print "%s, \t%.3f, \t%.3f" % ( j.GetName() , j.GetLimits()[0] , j.GetLimits()[1] ) 
        l = j.GetFirstAttached()
        if l is not None : links.append(l)
        l = j.GetSecondAttached()
        if l is not None : links.append(l)

    # Add addtitional links to remove from collision checking
    links.append(robot.GetLink('r_gripper_r_finger_link'))
    links.append(robot.GetLink('r_gripper_l_finger_tip_link'))

    for l in links : l.Enable(False)

    # Init collision checker
    collisionChecker = RaveCreateCollisionChecker( orEnv,'VoxelColChecker')
    collisionChecker.SendCommand('SetDimension robotcentered extent 2.0 2.5 2.0 offset -0.5 -1.25 0')
    collisionChecker.SendCommand('SetCollisionPointsRadii radii 10 0.22 0.15 0.14 0.1 0.1 0.1 0.1 0.1 0.1')
    collisionChecker.SendCommand('Initialize')
    orEnv.SetCollisionChecker(collisionChecker)

    print "Press return to run "
    sys.stdin.readline()

    collisionChecker.SendCommand('SetDrawing off')

    # ---------------------------------------------------------------
    # Set up motion planner
    indices = robot.GetManipulator("leftarm").GetArmIndices()
    robot.SetDOFValues( [2.0,0.5,0.5,-1.6,1.5,-1,0], indices )

    # Set active manipulator, Planning dofs
    indices = robot.GetManipulator("rightarm").GetArmIndices()

    q_init = [0.2,0.5,0.5,-0.6,-1.5,-1,0]
    q_goal = [0.2,-0.5,0.5,-0.6,-1.5,-1,0]
    
    robot.SetActiveDOFs(indices)
  
    # collChecker = orEnv.GetCollisionChecker()     

    prob = RaveCreateModule(orEnv, 'Move3d')
    orEnv.AddModule(prob, args='')

    prob.SendCommand('InitMove3dEnv')
    prob.SendCommand('LoadConfigFile /home/jmainpri/Dropbox/move3d/move3d-launch/parameters/params_pr2_shelf')
    prob.SendCommand('SetParameter jntToDraw 34')
    prob.SendCommand('SetParameter stompDrawIteration 1')

    robot.SetActiveDOFs(indices)

    while True:

        with robot:

            prob.SendCommand('RunStomp name ' + robot.GetName() +
                             ' jointgoals ' + SerializeConfig(q_goal) +
                             ' jointinits ' + SerializeConfig(q_init))

        trajectory = RaveCreateTrajectory(orEnv, "")
        trajectory.deserialize(open("traj_0.txt", 'r').read())
        robot.GetController().SetPath(trajectory)
        robot.WaitForController(0)
            
        print "Press return to exit."
        sys.stdin.readline()

        print orEnv.GetViewer().GetCameraTransform()
