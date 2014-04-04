#include "move3d-main.hpp"
#include <boost/thread.hpp>

#include "openrave_move3d_api.hpp"
#include "planner_functions.hpp"

// openrave --loadplugin libplugincpp.so --module mymodule "my args"

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

using namespace std;

#define FORIT(it, v) for(it = (v).begin(); it != (v).end(); (it)++)

Move3dProblem::Move3dProblem(EnvironmentBasePtr penv) : ProblemInstance(penv)
{
    __description = "THE MOVE3D PLUGIN";

    cout << __description << endl;

    RegisterCommand("initmove3denv",boost::bind(&Move3dProblem::InitMove3dEnv,this),"returns true if ok");
    RegisterCommand("loadconfigfile",boost::bind(&Move3dProblem::LoadConfigFile,this,_2),"returns true if ok");
    RegisterCommand("runrrt",boost::bind(&Move3dProblem::RunRRT,this),"returns true if ok");

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

bool Move3dProblem::RunRRT()
{
    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();

    cout << "robot name : " << robot->getName() << endl;
    cout << "nb dofs : " << robot->getNumberOfDofs() << endl;

    Move3D::confPtr_t q_init = robot->getNewConfig();
    Move3D::confPtr_t q_goal = robot->getNewConfig();

    (*q_init)[0] = 20.0;
    (*q_init)[1] = 50.0;

    (*q_goal)[0] = 200.0;
    (*q_goal)[1] = 700.0;

    or_runDiffusion( q_init, q_goal );

    RAVELOG_INFO("End RunRRT normally\n");

    return true;
}

bool Move3dProblem::SendCommand( std::ostream& sout, std::istream& sinput )
{
    ProblemInstance::SendCommand( sout, sinput );
    return true;
}

int Move3dProblem::main(const std::string& cmd)
{
    RAVELOG_DEBUG("env: %s\n", cmd.c_str());

    cout << "------------------------" << endl;
    cout << __PRETTY_FUNCTION__ << endl;

//    std::string file( "../ormodels/stones.env.xml" );
//    env_->Load( file );
//    InitMove3dEnv();
//    std::istringstream is( "/home/jmainpri/Dropbox/move3d/move3d-launch/parameters/params_stones" );
//    LoadConfigFile( is );
//    RunRRT();

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

