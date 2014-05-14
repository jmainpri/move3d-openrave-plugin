#ifndef MOVE3DMAIN_HPP
#define MOVE3DMAIN_HPP

#include <openrave/openrave.h>
#include <openrave/plugin.h>
#include <boost/bind.hpp>
#include <iostream>

#include <libmove3d/planners/API/Trajectory/trajectory.hpp>

using namespace OpenRAVE;

class Move3dProblem : public ModuleBase
{
public:
    Move3dProblem(EnvironmentBasePtr penv);
    virtual ~Move3dProblem();
    void Destroy();

    virtual int main( const std::string& args );
    virtual bool SendCommand( std::ostream& sout, std::istream& sinput );

    bool NumBodies( std::ostream& sout, std::istream& sinput );


private:

    bool InitMove3dEnv();
    bool LoadConfigFile( std::istream& sinput );
    bool SetParameter( std::istream& sinput );
    bool SetInitAndGoal( std::ostream& sout, std::istream& sinput );
    bool RunRRT( std::ostream& sout, std::istream& sinput );
    bool RunStomp( std::ostream& sout, std::istream& sinput );
    bool CreateTraj( Move3D::Trajectory* traj, RobotBasePtr robot, TrajectoryBasePtr ptraj );
    void PlayTrajectories( std::string robot_name, const std::vector<Move3D::Trajectory*>& trajs );
    bool CloneRobot( std::ostream& sout, std::istream& sinput );

    std::vector<dReal> goals_;
    std::vector<dReal> inits_;
    std::vector<EnvironmentBasePtr> envclones_;
//    std::vector<RobotBasePtr> robotclones_;
//    std::vector<dReal> starts_; // TODO (take a different start configuration as input)
};

#endif // MOVE3DMAIN_HPP
