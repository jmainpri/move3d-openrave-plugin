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

private:

    bool InitMove3dEnv();
    bool LoadConfigFile( std::istream& sinput );

    bool SetParameter( std::istream& sinput );
    bool SetInitAndGoal( std::ostream& sout, std::istream& sinput );
    bool SetPlayTrajectories( std::ostream& sout, std::istream& sinput );

    bool RunRRT( std::ostream& sout, std::istream& sinput );
    bool RunStomp( std::ostream& sout, std::istream& sinput );
    bool CreateTraj( Move3D::Trajectory* traj, RobotBasePtr robot, TrajectoryBasePtr ptraj, int id );
    void SaveAndPlayTrajectories( std::string robot_name, const std::vector<Move3D::Trajectory*>& trajs );
    bool CloneRobot( std::ostream& sout, std::istream& sinput );

    bool InitHumanPrediction( std::ostream& sout, std::istream& sinput );
    bool VoxelPrediction( std::ostream& sout, std::istream& sinput );

    std::vector<dReal> goals_; // Goal configuration
    std::vector<dReal> inits_; // Init configuration
    std::vector<EnvironmentBasePtr> envclones_; // Environment clones
    std::vector<bool> active_clones_; // Active clones for planning
    bool play_tajectories_; // Play trajectories after execution
    std::string current_directory_; // Current directory where the plugin is executed (used for trajecotry storing)

    bool init_human_prediction_; // set to true when the module is running

    // String converter
    template <class T>
    std::string ToStr(const T& value)
    {
        std::ostringstream oss;
        oss << value;
        return oss.str();
    }
};

#endif // MOVE3DMAIN_HPP
