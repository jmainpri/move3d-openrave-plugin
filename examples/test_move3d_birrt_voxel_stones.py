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

class TwoDPlanner():

    def __init__( self ):

        home_move3d = os.environ['HOME_MOVE3D']

        self.orEnv = Environment()
        self.orEnv.SetViewer('qtcoin')
        self.orEnv.GetViewer().EnvironmentSync()
        self.orEnv.Load( '../ormodels/stones.env.xml' )

        self.prob = RaveCreateModule( self.orEnv, 'Move3d' )
        self.orEnv.AddModule( self.prob, args='' )

        self.collisionChecker = RaveCreateCollisionChecker( self.orEnv,'VoxelColChecker')
        self.orEnv.SetCollisionChecker( self.collisionChecker ) 

        self.drawingHandles = []
        self.drawingHandles.append( misc.DrawAxes( self.orEnv, eye(4), 30 ) ) 

        self.prob.SendCommand('InitMove3dEnv')
        self.prob.SendCommand('LoadConfigFile ' + home_move3d + '../move3d-launch/parameters/params_stones')

    def run( self ) :

        self.prob.SendCommand('RunRRT')   

        print "Press return to exit."
        sys.stdin.readline()

    def SetCamera(self):
        T_cam =   ([[  2.44603788e-01,   7.28857907e-01,  -6.39480366e-01, 7.34846558e+02], \
                    [  9.60859728e-01,  -2.70673759e-01,   5.90279568e-02, 3.46425568e+02], \
                    [ -1.30067561e-01,  -6.28889392e-01,  -7.66538037e-01, 6.77975464e+02], \
                    [  0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
        # self.orEnv.GetViewer().GetCameraTransform()
        self.orEnv.GetViewer().SetCamera( T_cam )

if __name__ == "__main__":

    print "START OPENRAVE"
    planner = TwoDPlanner()
    planner.SetCamera()

    while True :
        planner.run()
