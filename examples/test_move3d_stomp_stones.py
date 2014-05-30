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

from misc_transform import *

class TwoDPlanner():

    def __init__(self):

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.GetViewer().EnvironmentSync()
        self.env.Load( '../ormodels/stones.env.xml' )
        self.env.SetDebugLevel(DebugLevel.Verbose)

        self.prob = RaveCreateModule( self.env, 'Move3d' )
        self.env.AddModule( self.prob, args='' )

        self.drawingHandles = []
        self.drawingHandles.append( misc.DrawAxes( self.env, eye(4), 30 ) ) 

        self.robot = self.env.GetRobots()[0]    

        # Disable Box0 link
        self.robot.GetLink('Box0').Enable( False )
 
        # Init collision checker
        collisionChecker = RaveCreateCollisionChecker( self.env,'VoxelColChecker')
        collisionChecker.SendCommand('SetDimension extent 500 800 30 voxelsize 5')
        collisionChecker.SendCommand('Initialize')
        collisionChecker.SendCommand('SetDrawing on')
        self.env.SetCollisionChecker(collisionChecker)

        # init trajectory
        self.trajectory = RaveCreateTrajectory(self.env, "")

        self.SetCamera() 

        print "Press return to run"
        sys.stdin.readline()

        collisionChecker.SendCommand('SetDrawing off')

        self.prob.SendCommand('InitMove3dEnv')
        self.prob.SendCommand('LoadConfigFile ../parameter_files/stomp_stones')
        self.prob.SendCommand('SetParameter jntToDraw 1')
        self.prob.SendCommand('SetParameter trajStompWithIterLimit 1')
        self.prob.SendCommand('SetParameter stompMaxIteration 300')

        self.collChecker = self.env.GetCollisionChecker()

    def run(self):

        q_init = [20, 50]
        q_goal = [200, 700]

        self.collChecker.SendCommand('SetDrawing off')
            
        with self.robot:

            self.prob.SendCommand('RunStomp name ' + self.robot.GetName() +
                                  ' jointgoals ' + SerializeConfig(q_goal) +
                                  ' jointinits ' + SerializeConfig(q_init))


        print self.env.GetViewer().GetCameraTransform()

    def SetCamera(self):

        T_cam = ([[  2.98714915e-01,   9.44717010e-01,  -1.35200486e-01,   2.94798950e+02],
                  [  9.53920362e-01,  -2.99784043e-01,   1.28635264e-02,   3.90384369e+02],
                  [ -2.83785562e-02,  -1.32813024e-01,  -9.90734757e-01,   7.22618408e+02],
                  [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,  1.00000000e+00]])
 
        self.env.GetViewer().SetCamera( T_cam )

if __name__ == "__main__":

    print "START OPENRAVE"
    planner = TwoDPlanner()
 
    while True:
        
        planner.run()

        planner.collChecker.SendCommand('SetDrawing on')

        planner.trajectory.deserialize(open("traj_0.txt", 'r').read())
        planner.robot.GetController().SetPath(planner.trajectory)
        planner.robot.WaitForController(0)

        print "Press return to run"
        sys.stdin.readline()

