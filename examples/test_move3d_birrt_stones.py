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
import os

class TwoDPlanner():

    def __init__( self ):

        home_move3d = os.environ['HOME_MOVE3D']

        self.env = Environment()
        self.env.SetViewer('qtcoin')
        self.env.GetViewer().EnvironmentSync()
        self.env.Load( '../ormodels/stones.env.xml' )

        self.robot = self.env.GetRobot('Puck')

        self.prob = RaveCreateModule( self.env, 'Move3d' )
        self.env.AddModule( self.prob, args='' )        
        self.prob.SendCommand('InitMove3dEnv')
        self.prob.SendCommand('LoadConfigFile ' + home_move3d + '/../move3d-launch/parameters/params_stones')
        self.prob.SendCommand('SetParameter drawScaleFactorNodeSphere 0.3')

    def run(self):

        q_init = [20, 50]
        q_goal = [200, 700]

        self.robot.SetDOFValues( q_init )

        with self.env:

            self.prob.SendCommand('RunRRT name ' + self.robot.GetName() +
                                  ' jointinits ' + SerializeConfig(q_init) +
                                  ' jointgoals ' + SerializeConfig(q_goal))

        trajectory = RaveCreateTrajectory(self.env, "")
        trajectory.deserialize(open("traj_0.txt", 'r').read())
        self.robot.GetController().SetPath(trajectory)
        self.robot.WaitForController(0)

    def SetCamera(self):
        T_cam = ([[2.44603788e-01,   7.28857907e-01,  -6.39480366e-01, 7.34846558e+02],
                  [9.60859728e-01,  -2.70673759e-01,   5.90279568e-02, 3.46425568e+02],
                  [-1.30067561e-01,  -6.28889392e-01,  -7.66538037e-01, 6.77975464e+02],
                  [0.00000000e+00,   0.00000000e+00,   0.00000000e+00, 1.00000000e+00]])
        # self.orEnv.GetViewer().GetCameraTransform()
        self.env.GetViewer().SetCamera( T_cam )

if __name__ == "__main__":

    print "START OPENRAVE"
    planner = TwoDPlanner()
    planner.SetCamera()

    while True :
        planner.run()
        print "Press return replan."
        sys.stdin.readline()
