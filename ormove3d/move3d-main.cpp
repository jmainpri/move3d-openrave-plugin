#include "move3d-main.hpp"
#include <boost/thread.hpp>

#include "openrave_move3d_api.hpp"
#include "planner_functions.hpp"

#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/scene.hpp>
#include <libmove3d/planners/planner/planEnvironment.hpp>

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
    RegisterCommand("clonerobot",boost::bind(&Move3dProblem::CloneRobot,this,_1,_2),"returns true if ok");
    RegisterCommand("setinitandgoal",boost::bind(&Move3dProblem::SetInitAndGoal,this,_1,_2),"returns true if ok");
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

    move3d_set_or_api_environment_pointer( GetEnv() );

    move3d_set_or_api_scene();
    move3d_set_or_api_functions_configuration();
    move3d_set_or_api_functions_localpath();
    move3d_set_or_api_functions_robot();
    move3d_set_or_api_functions_joint();
    move3d_set_or_api_collision_space();
    move3d_set_or_api_functions_draw();

    init_all_draw_functions_dummy();

    Move3D::global_Project = new Move3D::Project(new Move3D::Scene( GetEnv().get() ));
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

bool Move3dProblem::SetInitAndGoal( std::ostream& sout, std::istream& sinput )
{
    std::string name;
    std::string cmd;
    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "name" )
        {
            sinput >> name;
        }
        else if( cmd == "jointgoals" )
        {
            goals_.clear();
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
        else if( cmd == "jointinits" )
        {
            inits_.clear();
            cout << cmd << endl;
            // note that this appends to goals, does not overwrite them
            size_t temp;
            sinput >> temp;
            size_t oldsize = inits_.size();
            inits_.resize(oldsize+temp);
            for(size_t i = oldsize; i < oldsize+temp; i++) {
                sinput >> inits_[i];
                // cout << "goal[" << i << "] : " << goals_[i] << endl;
            }
        }
        else if( !sinput ) {
            RAVELOG_INFO("failed\n");
            sout << 0;
            return false;
        }
        else break;
    }

    // Get active dofs
    RobotBasePtr orRobot = GetEnv()->GetRobot( name );
    if( orRobot.get() == NULL ){
        RAVELOG_INFO( "could not find openrave robot with name : %s\n", name.c_str() );
        return false;
    }

    const std::vector<int>& indices = orRobot->GetActiveDOFIndices();

    // Get init configurations
    if( indices.size() != inits_.size() ) {
        RAVELOG_ERROR( "Error in setting init configuration, active %d, given %d\n", indices.size(), inits_.size() );
        return false;
    }

    // Get goal configurations
    if( indices.size() != goals_.size() ) {
        RAVELOG_ERROR( "Error in setting goal configuration, active %d, given %d\n", indices.size(), goals_.size() );
        return false;
    }

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getRobotByName( name );
    if( robot == NULL ){
        RAVELOG_INFO("No move3d robot with name %s\n", name.c_str() );
        return false;
    }

    Move3D::confPtr_t q_tmp1 = robot->getNewConfig();
    for(size_t i = 0; i < indices.size(); i++)
        (*q_tmp1)[ indices[i] ] = inits_[i];

    Move3D::confPtr_t q_tmp2 = robot->getNewConfig();
    for(size_t i = 0; i < indices.size(); i++)
        (*q_tmp2)[ indices[i] ] = goals_[i];

    robot->setInitPos( *q_tmp1 );
    robot->setGoalPos( *q_tmp2 );

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

bool Move3dProblem::CloneRobot( std::ostream& sout, std::istream& sinput )
{
    std::string name;
    size_t number = 0;
    std::string cmd;
    while(!sinput.eof())
    {
        sinput >> cmd;
        if( !sinput )
            break;

        if( cmd == "name" )
        {
            sinput >> name;
        }
        else if( cmd == "number" )
        {
            sinput >> number;
        }
        else break;
        if( !sinput ) {
            RAVELOG_DEBUG("failed\n");
            sout << 0;
            return false;
        }
    }

    // Remove all robot clones from environement
    for( size_t i=0; robotclones_.size(); i++) {
        cout << "Remove robot " << robotclones_[i]->GetName() << " from environment" << endl;
        GetEnv()->Remove( robotclones_[i] );
    }
    robotclones_.clear();

    if( number > 0 && number < 10 )
    {
        RobotBasePtr orRobot = GetEnv()->GetRobot( name );
        if( orRobot.get() != NULL )
        {
            for(size_t i=0; i<number; i++)
            {
                RobotBasePtr robot_clone = RaveCreateRobot( GetEnv() );
                robot_clone->Clone( orRobot, 0 );
                std::ostringstream convert;   // stream used for the conversion
                convert << i;      // insert the textual representation of 'Number' in the characters in the stream
                robot_clone->SetName( orRobot->GetName() + "_" + convert.str() );
                robot_clone->Enable( false );
                GetEnv()->Add( robot_clone );

                Move3D::Robot* new_move3d_robot = new Move3D::Robot( robot_clone.get() );
                new_move3d_robot->setUseLibmove3dStruct( false );
                Move3D::global_Project->getActiveScene()->insertRobot( new_move3d_robot );
            }

            std::vector<RobotBasePtr> robots;
            GetEnv()->GetRobots( robots );
            for(size_t i = 0; i<robots.size(); i++)
                cout << "robots->GetName() is : "  << robots[i]->GetName() << endl;
        }
        else {
            cout << "Robot name : " << name << " counld not be found in enviroment" << endl;
            return false;
        }
    }
    else {
        cout << "Wrong number of robots" << endl;
        return false;
    }

    return true;
}

bool Move3dProblem::RunRRT( std::ostream& sout, std::istream& sinput )
{
    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    if( !SetInitAndGoal( sout, sinput ) ){
        RAVELOG_INFO("Error in set init and goal\n");
        return false;
    }

    move3d_draw_clear();

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();
    cout << "robot name : " << robot->getName() << " . nb dofs : " << robot->getNumberOfDofs() << endl;

    // Start RRT
    Move3D::Trajectory* traj = or_runDiffusion( robot->getInitPos(), robot->getGoalPos() );

    if( traj != NULL ){
        RobotBasePtr orRobot = GetEnv()->GetRobot( robot->getName() );
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

    if( !SetInitAndGoal( sout, sinput ) ){
        RAVELOG_INFO("Error in set init and goal\n");
        return false;
    }

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();
    cout << "robot name : " << robot->getName() << " . nb dofs : " << robot->getNumberOfDofs() << endl;

    // Start Stomp
    Move3D::Trajectory* traj = or_runStomp( robot->getInitPos(), robot->getGoalPos() );

    if( traj != NULL ){
        RobotBasePtr orRobot = GetEnv()->GetRobot( robot->getName() );
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
        GetEnv()->Load( file );
        InitMove3dEnv();
        std::istringstream is( "../parameter_files/stomp_stones" );
        LoadConfigFile( is );
        // RunStomp();
    }

    if( cmd == "run_test_2" )
    {
        std::string filename;
        filename = "robots/pr2-beta-static.zae";
        GetEnv()->Load( filename );
        filename = "data/shelf.kinbody.xml";
        GetEnv()->Load( filename );

        KinBodyPtr shelf = GetEnv()->GetKinBody( "Shelf" );
        TransformMatrix T;

        T.m[0]= 1.00000000e+00;   T.m[1]=0.00000000e+00;   T.m[2]=0.00000000e+00;
        T.m[4]= 0.00000000e+00;   T.m[5]=1.11022302e-16;   T.m[6]=1.00000000e+00;
        T.m[8]= 0.00000000e+00;   T.m[9]=-1.00000000e+00;  T.m[10]=1.11022302e-16;

        T.trans.x=7.00000000e-01;  T.trans.y=-5.00000000e-01;  T.trans.z=0.00000000e+00;

        shelf->SetTransform( T );

        RobotBasePtr robot = GetEnv()->GetRobot( "pr2" );
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

    if( cmd == "run_test_3" )
    {
        std::string filename;
        filename = "/home/jmainpri/catkin_ws_hrics/src/NRI-Human-Robot-Collaboration/hrcol_motion_planning/models/data/IRG_lab_with_human.env.xml";
        GetEnv()->Load( filename );

        RobotBasePtr robot = GetEnv()->GetRobot("ABBIE");
        RobotBasePtr human = GetEnv()->GetRobot("HERAKLES_HUMAN");

        TransformMatrix T = human->GetTransform();
        T.trans.x=-1;  T.trans.y=1;  T.trans.z=0;
        human->SetTransform(T);

        std::string coll_checker_name = "VoxelColChecker" ;
        CollisionCheckerBasePtr pchecker = RaveCreateCollisionChecker( GetEnv(), coll_checker_name.c_str() ); // create the module
        if( !pchecker ) {
            RAVELOG_ERROR( "Failed to create checker %s\n", coll_checker_name.c_str() );
            return false;
        }

        std::ostringstream out;
        std::istringstream iss;

        // VOXEL COLLISON CHECKER
        iss.clear(); iss.str("SetDimension robotcentered extent 2.0 2.0 2.0 offset 1.0 1.0 -1.0"); pchecker->SendCommand( out, iss );
        iss.clear(); iss.str("SetCollisionPointsRadii radii 6 0.20 .14 .10 .08 .07 .05 activation 6 0 1 1 1 1 0"); pchecker->SendCommand( out, iss );
        iss.clear(); iss.str("Initialize"); pchecker->SendCommand( out, iss );
        iss.clear(); iss.str("SetDrawing off"); pchecker->SendCommand( out, iss );

        GetEnv()->SetCollisionChecker( pchecker );

        // MOVE3D
        iss.clear(); iss.str("InitMove3dEnv"); SendCommand( out, iss );
        iss.clear(); iss.str("LoadConfigFile /home/jmainpri/Dropbox/move3d/move3d-launch/parameters/params_pr2_shelf"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter drawTraj 1"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter jntToDraw 5"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter stompDrawIteration 1"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter trajStompTimeLimit 2.5"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter trajOptimSmoothWeight 1000.0"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter trajOptimObstacWeight 10.0"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter trajOptimStdDev 0.002"); SendCommand( out, iss );

        iss.clear(); iss.str("SetParameter trajStompRunParallel 1"); SendCommand( out, iss );
        iss.clear(); iss.str("SetParameter trajStompRunMultiple 1"); SendCommand( out, iss );
        iss.clear(); iss.str("CloneRobot name ABBIE number 3"); SendCommand( out, iss );

        iss.clear(); iss.str("SetInitAndGoal name ABBIE_0 jointinits 6 2.303 1.627 -0.168 -0.044 0.465 0.608 jointgoals 6 1.263 1.385 -0.609 -0.001 0.813 0.608"); SendCommand( out, iss );
        iss.clear(); iss.str("SetInitAndGoal name ABBIE_1 jointinits 6 2.303 1.627 -0.168 -0.044 0.465 0.608 jointgoals 6 0.660 1.385 -0.385 -0.001 0.465 0.608"); SendCommand( out, iss );
        iss.clear(); iss.str("SetInitAndGoal name ABBIE_2 jointinits 6 2.303 1.627 -0.168 -0.044 0.465 0.608 jointgoals 6 0.197 1.524 -0.914 -0.001 0.813 0.608"); SendCommand( out, iss );

        iss.clear(); iss.str("RunStomp name ABBIE jointinits 6 2.303 1.627 -0.168 -0.044 0.465 0.608 jointgoals 6 0.197 1.524 -0.914 -0.001 0.813 0.608"); SendCommand( out, iss );
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

