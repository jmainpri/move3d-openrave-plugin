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

    orEnv.GetViewer().SetCamera( T_cam )

    T = eye(4)
    T[0,3] = 0.0
    T[1,3] = 0.0
    T[2,3] = 0.0

    for b in orEnv.GetBodies() :
        print "name : ", b.GetName()
        if  b.GetName() == "Shelf":
            shelf = b

    # Get body
    shelf.SetTransform( array( MakeTransform( rodrigues([-pi/2,0,0]), matrix([0.7,-0.5,0]) ) ) )

    # Get robot
    robot = orEnv.GetRobots()[0]
    robot.SetTransform( T )

    indices = robot.GetManipulator( "leftarm" ).GetArmIndices()
    robot.SetDOFValues( [2.0,0.5,0.5,-1.6,1.5,-1,0], indices )

    # Set active manipulator
    indices = robot.GetManipulator( "rightarm" ).GetArmIndices()

    # Print the limits
    for jIdx, j in enumerate(robot.GetJoints()):
        if jIdx not in indices :
            continue
        print "%d, %s, \t%.3f, \t%.3f" % ( jIdx, j.GetName() , j.GetLimits()[0] , j.GetLimits()[1] )

    q_init = [0.2,0.5,0.5,-0.6,-1.5,-1,0]
    q_goal = [0.2,-0.3,0.5,-0.6,-1.5,-1,0]
    
    robot.SetActiveDOFs( indices )
    robot.SetDOFValues( q_init, indices )
    
    print "Press return to run "
    sys.stdin.readline()

    prob = RaveCreateModule( orEnv, 'Move3d' )
    orEnv.AddModule( prob, args='' )

    prob.SendCommand('InitMove3dEnv')
    prob.SendCommand('LoadConfigFile /home/jmainpri/Dropbox/move3d/move3d-launch/parameters/params_pr2_shelf')
    prob.SendCommand('SetParameter jntToDraw 33')
        
    collChecker = orEnv.GetCollisionChecker()

    while True :

        prob.SendCommand('RunStomp jointgoals ' + SerializeConfig( q_goal ) )   
        orEnv.SetCollisionChecker( collChecker )
        print "Press return to exit."
        sys.stdin.readline()

        print orEnv.GetViewer().GetCameraTransform()
