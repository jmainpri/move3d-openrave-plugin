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

    def __init__( self ):

        self.orEnv = Environment()
        self.orEnv.SetViewer('qtcoin')
        self.orEnv.GetViewer().EnvironmentSync()
        self.orEnv.Load( '../ormodels/stones.env.xml' )
        self.orEnv.SetDebugLevel(DebugLevel.Verbose)

        self.prob = RaveCreateModule( self.orEnv, 'Move3d' )
        self.orEnv.AddModule( self.prob, args='' )

        self.drawingHandles = []
        self.drawingHandles.append( misc.DrawAxes( self.orEnv, eye(4), 30 ) ) 

        self.robot = self.orEnv.GetRobots()[0]

        self.prob.SendCommand('InitMove3dEnv')
        self.prob.SendCommand('LoadConfigFile ../parameter_files/stomp_stones')

        self.collChecker = self.orEnv.GetCollisionChecker()

    def run( self ) :

        q_init = [20,50]
        q_goal = [200,700]

        self.robot.SetDOFValues( array(q_init) )
        self.prob.SendCommand('RunStomp jointgoals ' + SerializeConfig(q_goal) )

        self.orEnv.SetCollisionChecker( self.collChecker )

        print "Press return to run"
        sys.stdin.readline()

        print self.orEnv.GetViewer().GetCameraTransform()

    def SetCamera(self):

        T_cam = ([[  2.98714915e-01,   9.44717010e-01,  -1.35200486e-01,   2.94798950e+02],
                  [  9.53920362e-01,  -2.99784043e-01,   1.28635264e-02,   3.90384369e+02],
                  [ -2.83785562e-02,  -1.32813024e-01,  -9.90734757e-01,   7.22618408e+02],
                  [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00,  1.00000000e+00]])
 
        self.orEnv.GetViewer().SetCamera( T_cam )

if __name__ == "__main__":

    print "START OPENRAVE"
    planner = TwoDPlanner()
    planner.SetCamera() 

    # print "Press return to run"
    # sys.stdin.readline()

    while True :
        planner.run()
