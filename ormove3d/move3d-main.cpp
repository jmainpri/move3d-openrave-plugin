#include "move3d-main.hpp"
#include <boost/thread.hpp>

#include "openrave_move3d_api.hpp"
#include "planner_functions.hpp"

#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/planners/API/project.hpp>

#define UNIX

#include <Move3D-Qt-Gui/src/qtMainInterface/settings.hpp>
#include <Move3D-Qt-Gui/src/planner_handler.hpp>

#include <libmove3d/graphic/proto/g3d_newWindow.hpp>

//! move3d studio functions (remove when transfering loading function
//-----------------------------------------------
std::string move3d_studio_settings_file = ".save_interface_params";
void draw_opengl() {}
PlannerHandler* global_plannerHandler(NULL);
class MainWindow;
MainWindow* global_w(NULL);
//-----------------------------------------------

using std::cout;
using std::endl;

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

Move3dProblem::Move3dProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "THE MOVE3D PLUGIN";

    cout << __description << endl;

    RegisterCommand("initmove3denv",boost::bind(&Move3dProblem::InitMove3dEnv,this),"returns true if ok");
    RegisterCommand("loadconfigfile",boost::bind(&Move3dProblem::LoadConfigFile,this,_2),"returns true if ok");
    RegisterCommand("setparameter",boost::bind(&Move3dProblem::SetParameter,this,_2),"returns true if ok");
    RegisterCommand("runrrt",boost::bind(&Move3dProblem::RunRRT,this,_1,_2),"returns true if ok");
    RegisterCommand("runstomp",boost::bind(&Move3dProblem::RunStomp,this,_1,_2),"returns true if ok");

    env_ = penv;
}

void Move3dProblem::Destroy()
{
    RAVELOG_INFO("module unloaded from environment\n");
}

Move3dProblem::~Move3dProblem()
{

}

bool Move3dProblem::InitMove3dEnv()
{
    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    move3d_set_or_api_environment_pointer( env_ );

    move3d_set_or_api_scene();
    move3d_set_or_api_functions_configuration();
    move3d_set_or_api_functions_localpath();
    move3d_set_or_api_functions_robot();
    move3d_set_or_api_functions_joint();
    move3d_set_or_api_collision_space();
    move3d_set_or_api_functions_draw();

    init_all_draw_functions_dummy();

    Move3D::global_Project = new Move3D::Project(new Move3D::Scene( env_.get() ));
    //    return ( Move3D::global_Project != NULL );
    return true;
}

bool Move3dProblem::LoadConfigFile( std::istream& sinput )
{
    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    std::string cmd;
    sinput >> cmd;

    cout << "Load file : " << cmd <<  "'" << endl;

    qt_loadInterfaceParameters( false, cmd, false );

    return true;
}

bool Move3dProblem::SetParameter( std::istream& sinput )
{
//    cout << "------------------------" << endl;
//    cout << __PRETTY_FUNCTION__ << endl;

    std::string name;
    double value;

    sinput >> name;
    if( !sinput )
        return false;

    sinput >> value;

    cout << "Set parameter name : " << name <<  " , value : " << value << endl;

    qt_setParameter( name, value );

    return true;
}

bool Move3dProblem::GetOptions( std::ostream& sout, std::istream& sinput )
{
    goals_.clear();

    std::string cmd;
    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "jointgoals" )
        {
            cout << cmd << endl;
            // note that this appends to goals, does not overwrite them
            size_t temp;
            sinput >> temp;
            size_t oldsize = goals_.size();
            goals_.resize(oldsize+temp);
            for(size_t i = oldsize; i < oldsize+temp; i++) {
                sinput >> goals_[i];
                // cout << "goal[" << i << "] : " << goals_[i] << endl;
            }
        }
        else break;
        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            sout << 0;
            return false;
        }
    }

    return true;
}

bool Move3dProblem::CreateTraj( Move3D::Trajectory* traj, RobotBasePtr robot, TrajectoryBasePtr ptraj )
{
    ConfigurationSpecification confspec =  robot->GetActiveConfigurationSpecification();
    ConfigurationSpecification::Group& group = confspec.GetGroupFromName("joint_values");
    group.interpolation = "linear";
    confspec.AddDerivativeGroups( 0, true ); // Add joint_velocity

    cout << confspec << endl;

    ptraj->Init( confspec );

    std::vector<dReal> vtraj_data;
    const std::vector<int>& indices = robot->GetActiveDOFIndices();

    for( int j=0; j<traj->getNbOfViaPoints(); j++)
    {
        Move3D::confPtr_t q = (*traj)[j];

        for( size_t i=0; i<indices.size(); i++) // Positions
        {
            vtraj_data.push_back( (*q)[ indices[i] ] );
        }

//        for( size_t i=0; i<indices.size(); i++) // Vellocity
//        {
//            vtraj_data.push_back( 0.1 );
//        }

        vtraj_data.push_back( 0.2 ); // Time stamp
    }

    ptraj->Insert( ptraj->GetNumWaypoints(), vtraj_data, confspec );

    if( int( ptraj->GetNumWaypoints() ) != traj->getNbOfViaPoints() )
    {
        RAVELOG_INFO("Error, trajectory timer changed the number of points: %d before vs. %d after\n", traj->getNbOfViaPoints(), ptraj->GetNumWaypoints() );
        return false;
    }

    cout << "traj number of way points : " << ptraj->GetNumWaypoints() << endl;

    // Save to file
    std::string filename( "traj.txt" );
    std::ofstream outfile( filename.c_str(), std::ios::out );
    outfile.precision(16);
    ptraj->serialize( outfile );
    outfile.close();
    // chmod( filename.c_str(), S_IRWXG | S_IRWXO | S_IRWXU ); //chmod 777

    return true;
}

bool Move3dProblem::RunRRT( std::ostream& sout, std::istream& sinput )
{
    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    if( !GetOptions( sout, sinput ) ){
        return false;
    }

    move3d_draw_clear();

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();
    cout << "robot name : " << robot->getName() << " . nb dofs : " << robot->getNumberOfDofs() << endl;

    Move3D::confPtr_t q_init = robot->getCurrentPos();
    Move3D::confPtr_t q_goal = robot->getNewConfig();

    // Set goal configurations
    RobotBasePtr orRobot = env_->GetRobot( robot->getName() );
    const std::vector<int>& indices = orRobot->GetActiveDOFIndices();
    if( indices.size() != goals_.size() ) {
        RAVELOG_ERROR( "Error in setting goal configuration, active %d, given %d\n", indices.size(), goals_.size() );
        return false;
    }

    for(size_t i = 0; i < indices.size(); i++)
        (*q_goal)[ indices[i] ] = goals_[i];

    Move3D::Trajectory* traj = or_runDiffusion( q_init, q_goal );

    if( traj != NULL ){
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        CreateTraj( traj, orRobot, ptraj );
        delete traj;
        orRobot->GetController()->SetPath(ptraj);
    }

    RAVELOG_INFO("End RunRRT normally\n");
    return true;
}

bool Move3dProblem::RunStomp( std::ostream& sout, std::istream& sinput )
{
    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    if( !GetOptions( sout, sinput ) ){
        return false;
    }

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();
    cout << "robot name : " << robot->getName() << " . nb dofs : " << robot->getNumberOfDofs() << endl;

    Move3D::confPtr_t q_init = robot->getCurrentPos();
    Move3D::confPtr_t q_goal = robot->getNewConfig();

    // Set goal configurations
    RobotBasePtr orRobot = env_->GetRobot( robot->getName() );
    const std::vector<int>& indices = orRobot->GetActiveDOFIndices();
    if( indices.size() != goals_.size() ) {
        RAVELOG_ERROR( "Error in setting goal configuration, active %d, given %d\n", indices.size(), goals_.size() );
        return false;
    }

    for(size_t i = 0; i < indices.size(); i++)
        (*q_goal)[ indices[i] ] = goals_[i];

    // Set Collision Checker
//    std::string coll_checker_name = "VoxelColChecker" ;
//    CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker( GetEnv(), coll_checker_name.c_str() ); // create the module
//    if( !pchecker ) {
//        RAVELOG_ERROR( "Failed to create checker %s\n", coll_checker_name.c_str() );
//        return false;
//    }
//    GetEnv()->SetCollisionChecker( pchecker );

    // Start Stomp
    Move3D::Trajectory* traj = or_runStomp( q_init, q_goal );

    if( traj != NULL ){
        TrajectoryBasePtr ptraj = RaveCreateTrajectory(GetEnv(),"");
        CreateTraj( traj, orRobot, ptraj );
        delete traj;
        orRobot->GetController()->SetPath(ptraj);
    }

    RAVELOG_INFO("End Stomp normally\n");
    return true;
}

bool Move3dProblem::SendCommand( std::ostream& sout, std::istream& sinput )
{
    ProblemInstance::SendCommand( sout, sinput );
    return true;
}

// To run in gdb use the following command line
// gdb --args openrave --loadplugin ../plugins/libor-move3d.so --module move3d "run_test_2"
int Move3dProblem::main(const std::string& cmd)
{
    RAVELOG_DEBUG("env: %s\n", cmd.c_str());

    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    if( cmd == "run_test_1" )
    {
        std::string file( "../ormodels/stones.env.xml" );
        env_->Load( file );
        InitMove3dEnv();
        std::istringstream is( "../parameter_files/stomp_stones" );
        LoadConfigFile( is );
        // RunStomp();
    }

    if( cmd == "run_test_2" )
    {
        std::string filename;
        filename = "robots/pr2-beta-static.zae";
        env_->Load( filename );
        filename = "data/shelf.kinbody.xml";
        env_->Load( filename );

        KinBodyPtr shelf = env_->GetKinBody( "Shelf" );
        TransformMatrix T;

        T.m[0]= 1.00000000e+00;   T.m[1]=0.00000000e+00;   T.m[2]=0.00000000e+00;
        T.m[4]= 0.00000000e+00;   T.m[5]=1.11022302e-16;   T.m[6]=1.00000000e+00;
        T.m[8]= 0.00000000e+00;   T.m[9]=-1.00000000e+00;  T.m[10]=1.11022302e-16;

        T.trans.x=7.00000000e-01;  T.trans.y=-5.00000000e-01;  T.trans.z=0.00000000e+00;

        shelf->SetTransform( T );

        RobotBasePtr robot = env_->GetRobot( "pr2" );
        std::vector<int> indices;
        indices.push_back(27);
        indices.push_back(28);
        indices.push_back(29);
        indices.push_back(30);
        indices.push_back(31);
        indices.push_back(32);
        indices.push_back(33);
        robot->SetActiveDOFs( indices );

        std::vector<dReal> q_init;
        q_init.push_back(0.2);
        q_init.push_back(0.5);
        q_init.push_back(0.5);
        q_init.push_back(-0.6);
        q_init.push_back(-1.5);
        q_init.push_back(-1);
        q_init.push_back(0);
        robot->SetActiveDOFValues( q_init );

        InitMove3dEnv();
        std::istringstream is( "../parameter_files/pr2_shelf" );
        LoadConfigFile( is );

        std::ostringstream out;
        std::istringstream q_goal( "jointgoals 7 0.2 -0.3 0.5 -0.6 -1.5 -1 0" );
        RunStomp( out, q_goal );
    }

    //    const char* delim = " \r\n\t";
    //    string mycmd = cmd;
    //    char* p = strtok(&mycmd[0], delim);
    //    if( p != NULL )
    //        strRobotName_ = p;
    //    cout << "strRobotName_: " << strRobotName_ << endl;

    //std::vector<RobotBasePtr> robots;
    //GetEnv()->GetRobots(robots);
    //SetActiveRobots(robots);
    return 0;
}

//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------

InterfaceBasePtr CreateInterfaceValidated(InterfaceType type, const std::string& interfacename, std::istream& sinput, EnvironmentBasePtr penv)
{
    if( type == PT_ProblemInstance && interfacename == "move3d" ) {
        return InterfaceBasePtr(new Move3dProblem(penv));
    }
    return InterfaceBasePtr();
}

void GetPluginAttributesValidated(PLUGININFO& info)
{
    info.interfacenames[PT_ProblemInstance].push_back("Move3d");
}

RAVE_PLUGIN_API void DestroyPlugin()
{
    RAVELOG_INFO("destroying plugin\n");
}

