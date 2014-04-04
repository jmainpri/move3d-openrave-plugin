#!/usr/bin/env python
# Jim Mainprice, ARC
# March 2013
# Worcester Polytechnic Institute

# openrave-robot.py /your/path/to/your.robot.xml --info=joints
# On that page you can find more examples on how to use openrave-robot.py.
# http://openrave.org/docs/latest_stable/command_line_tools/

from openravepy import *
from numpy import *

class TwoDPlanner():

    def __init__( self ):

        self.orEnv = Environment()
        self.orEnv.SetViewer('qtcoin')
        self.orEnv.GetViewer().EnvironmentSync()
        self.orEnv.Load( '../ormodels/stones.env.xml' )

    def run( self ) :

        robot = self.orEnv.GetRobots()[0]

        # Set the initial configuration
        indices = robot.GetActiveManipulator().GetArmIndices()
        robot.SetDOFValues( [20.0,50.0], indices )

        # Embeded way of computing an RRT
        # prob = interfaces.BaseManipulation( robot ) # create the interface for basic manipulation programs
        # prob.MoveManipulator( goal=[200.0,700.0] ) # call motion planner with goal joint angles
        # robot.WaitForController(0) # wait

        planner = RaveCreatePlanner( self.orEnv, 'birrt' )
        params = Planner.PlannerParameters()
        params.SetRobotActiveJoints( robot )

        # Set the goal configuration
        params.SetGoalConfig( [200.0,700.0] )
        planner.InitPlan( robot, params )

        traj = RaveCreateTrajectory( self.orEnv, '' )

        # Number of configurations drawing the path
        nb_of_config = 30

        if planner.PlanPath(traj) :
            for t in linspace( 0, traj.GetDuration(), nb_of_config ):
                newrobot = RaveCreateRobot( self.orEnv, robot.GetXMLId() )
                newrobot.Clone( robot, 0 )
                for link in newrobot.GetLinks():
                    for geom in link.GetGeometries():
                        geom.SetTransparency( 0.3 )
                self.orEnv.Add( newrobot, True )
                newrobot.SetDOFValues( traj.Sample( t )[indices] , indices )
        i = 0
        while( True ):
            i += 1

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
    planner.run()
