#include "planner_functions.hpp"

#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/planners/API/project.hpp>
#include <libmove3d/planners/API/Roadmap/graph.hpp>

#include <libmove3d/planners/planner/planEnvironment.hpp>
#include <libmove3d/planners/planner/planner.hpp>
#include <libmove3d/planners/planner/Diffusion/RRT.hpp>
#include <libmove3d/planners/planner/Diffusion/EST.hpp>
#include <libmove3d/planners/planner/Diffusion/Variants/Transition-RRT.hpp>
#include <libmove3d/planners/planner/Diffusion/Variants/Multi-RRT.hpp>
#include <libmove3d/planners/planner/Diffusion/Variants/Multi-TRRT.hpp>
#include <libmove3d/planners/planner/Diffusion/Variants/Threshold-RRT.hpp>
#include <libmove3d/planners/planner/Diffusion/Variants/Star-RRT.hpp>
#include <libmove3d/planners/planner/TrajectoryOptim/trajectoryOptim.hpp>
#include <libmove3d/planners/planner/TrajectoryOptim/Classic/costOptimization.hpp>
#include <libmove3d/planners/planner/TrajectoryOptim/trajectoryOptim.hpp>
#include <libmove3d/planners/planner/TrajectoryOptim/Stomp/run_parallel_stomp.hpp>

#include <libmove3d/planners/collision_space/collision_space_factory.hpp>
//#include <libmove3d/planners/planner/plannerFunctions.hpp>

#define UNIX

#include <libmove3d/include/Planner-pkg.h>
#include <libmove3d/p3d/env.hpp>

#include <iostream>
#include <sys/time.h>

using std::cout;
using std::cerr;
using std::endl;

using namespace Move3D;

static unsigned int runId = 0;
static unsigned int trajId = 0;

struct RRTStatistics
{
    int runId;
    bool succeeded;
    double time;
    double cost;
    int nbNodes;
    int nbExpansions;
};

static RRTStatistics rrt_statistics;
static TrajectoryStatistics traj_statistics;

static bool set_costspace = false;

Trajectory* move3d_extract_traj( bool is_traj_found, int nb_added_nodes, Graph* graph, confPtr_t q_source, confPtr_t q_target)
{
    cout << __PRETTY_FUNCTION__ << endl;

    Move3D::Trajectory* traj = NULL;
    // Robot* rob = graph->getRobot();

    // If traj is found, extract it from the graph
    if (/*rrt->trajFound()*/ is_traj_found )
    {
        // Case of direct connection
        if( nb_added_nodes == 2 )
        {
            std::vector<confPtr_t> configs;

            configs.push_back( q_source );
            configs.push_back( q_target );

            cout << "Creating trajectory from two confgurations" << endl;
            traj = new Move3D::Trajectory( configs );
        }
        else
        {
            //traj = graph->extractBestTraj( q_source, q_target ); // Old extract
            traj = graph->extractAStarShortestPathsTraj( q_source, q_target );
        }
    }

    trajId++;

    cout << "compute traj cost" << endl;

    // Return trajectory or NULL if falses
    if (traj)
    {
        //traj->costDeltaAlongTraj();
        if( PlanEnv->getBool(PlanParam::trajComputeCostAfterPlannif) )
        {
            bool is_cost_space = ENV.getBool(Env::isCostSpace);
            ENV.setBool(Env::isCostSpace, true);

            traj->resetCostComputed();
            traj->costStatistics( traj_statistics );

            cout << "--- stats on traj ---" << endl;
            cout << " length = " << traj_statistics.length << endl;
            cout << " max = " << traj_statistics.max << endl;
            cout << " average = " << traj_statistics.average << endl;
            cout << " integral = " << traj_statistics.integral << endl;
            cout << " mecha_work = " << traj_statistics.mecha_work << endl;
            cout << "---------------------" << endl;

            // Compute traj cost
            rrt_statistics.cost = traj->cost();
            cout << "is_cost_space : " << ENV.getBool(Env::isCostSpace) << " , traj_cost : " << rrt_statistics.cost << endl ;
            ENV.setBool(Env::isCostSpace, is_cost_space);
        }

        cout << "nb of paths = " << traj->getNbOfPaths() << endl;
        cout << "max param = " << traj->getRangeMax() << endl;

        return traj;
    }
    else
    {
        cout << __FILE__ << " , " << __PRETTY_FUNCTION__ << " : No traj found" << endl;
        return NULL;
    }
}

// ---------------------------------------------------------------------------------
// Allocates an RRT depending on env variables
// ---------------------------------------------------------------------------------
RRT* move3d_allocate_rrt( Robot* rob,Graph* graph )
{
    cout << __PRETTY_FUNCTION__ << endl;

    RRT* rrt;

//    if(ENV.getBool(Env::isManhattan))
//    {
//        rrt = new ManhattanLikeRRT(rob,graph);
//    }
    if(ENV.getBool(Env::isMultiRRT) && ENV.getBool(Env::isCostSpace)  )
    {
        rrt = new MultiTRRT(rob,graph);
    }
    else if(ENV.getBool(Env::isMultiRRT))
    {
        rrt = new MultiRRT(rob,graph);
    }
#ifdef HRI_COSTSPACE
    else if(ENV.getBool(Env::HRIPlannerWS) && ENV.getBool(Env::HRIPlannerTRRT))
    {
        rrt = new HRICS::HRICS_RRT(rob,graph);
    }
    else if(ENV.getBool(Env::HRIPlannerCS) && ENV.getBool(Env::HRIPlannerTRRT))
    {
        rrt = new HRICS::HRICS_RRTPlan(rob,graph);
    }
#endif
    else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::costThresholdRRT) )
    {
        rrt = new ThresholdRRT(rob,graph);
    }
    else if(ENV.getBool(Env::isCostSpace) && PlanEnv->getBool(PlanParam::starRRT) )
    {
        rrt = new StarRRT(rob,graph);
    }
    else if(ENV.getBool(Env::isCostSpace) && ENV.getBool(Env::useTRRT) )
    {
        rrt = new TransitionRRT(rob,graph);
    }
    else
    {
        if( ENV.getBool(Env::isCostSpace) && (!ENV.getBool(Env::useTRRT)) )
        {
            ENV.setBool(Env::isCostSpace,false);
            set_costspace = true;
        }

        rrt = new RRT(rob,graph);
    }

    return rrt;
}

Trajectory* move3d_planner_function( Robot* rob, confPtr_t q_source, confPtr_t q_target )
{
    cout << "* PLANNING ***************" << endl;

    q_source->print();
    q_target->print();

    ChronoTimeOfDayOn();

    // Allocate the p3d_graph if does't exist
    // Delete graph if it exists, creates a new graph , Allocate RRT
    delete API_activeGraph;
    Graph* graph = API_activeGraph = new Graph(rob);

    // Delete last global planner
    if( global_Move3DPlanner ) delete global_Move3DPlanner;

    // Main Run functions of all RRTs
    // All RRTs are initilized with init and run here
    int nb_added_nodes = 0;
    RRT* rrt = move3d_allocate_rrt( rob, graph );
    global_Move3DPlanner = rrt;
    nb_added_nodes += rrt->setInit(q_source);
    nb_added_nodes += rrt->setGoal(q_target);
    nb_added_nodes += rrt->init();
    rrt->setInitialized(true);
    rrt->setRunId( runId );

    cout << "rrt->run()" << endl;

    nb_added_nodes += rrt->run();

    if ((rrt->getNumberOfExpansion() - rrt->getNumberOfFailedExpansion() + rrt->getNumberOfInitialNodes())
            != graph->getNumberOfNodes() )
    {
        cout << "Error in RRT nb of expansion ";
        cout << "compared to initial nb of nodes in graph total nb of nodes in the graph" << endl;
    }

    graph->getGraphStruct()->totTime = graph->getGraphStruct()->rrtTime;

    double time;
    ChronoTimeOfDayTimes(&time);
    cout << "Time before trajectory extraction :"  << time << " sec." << endl;

    // Extract the trajectory if one exists, else return NULL
    Trajectory* traj = move3d_extract_traj( rrt->trajFound(), nb_added_nodes, graph, q_source, q_target );

    ChronoTimeOfDayTimes(&time);
    ChronoTimeOfDayOff();

    cout << "** ** --------------------------" << endl;

    if( traj ) {
        cout << "Success" << endl;
    }
    else {
        cout << "Fail" << endl;
    }

    cout << "TIME ="  << time << " sec." << endl;
    cout << "NB NODES " << graph->getNumberOfNodes() << endl;
    cout << "NB EXPANSION " << rrt->getNumberOfExpansion() << endl;
    cout << "** ** --------------------------" << endl;

    rrt_statistics.runId = rrt->getRunId();
    rrt_statistics.succeeded = (traj!=NULL);
    rrt_statistics.time = time;
    //rrt_statistics.cost = 0.0;
    rrt_statistics.nbNodes = graph->getNumberOfNodes();
    rrt_statistics.nbExpansions = rrt->getNumberOfExpansion();

    if( set_costspace )
        ENV.setBool(Env::isCostSpace,true);

    return traj;
}

void move3d_smoothing_function( Trajectory& traj, int nbSteps, double maxTime )
{
    cout << "** SMOOTHING ***************" << endl;

    if( maxTime != -1 )
        PlanEnv->setDouble( PlanParam::timeLimitSmoothing, maxTime );

    if(PlanEnv->getBool(PlanParam::withDeformation) || PlanEnv->getBool(PlanParam::withShortCut) )
    {
        double optTime = 0.0;

        cout << "nb of paths = " << traj.getNbOfPaths() << endl;
        cout << "range max = " << traj.getRangeMax() << endl;

        Move3D::CostOptimization optimTrj( traj );

        optimTrj.setRunId( runId );
        optimTrj.setContextName( ENV.getString(Env::nameOfFile) );

        if(PlanEnv->getBool(PlanParam::withDeformation))
        {
            optimTrj.resetCostComputed();
            optimTrj.runDeformation( nbSteps , runId );
            optTime += optimTrj.getTime();
        }

        if(PlanEnv->getBool(PlanParam::withShortCut))
        {
            optimTrj.resetCostComputed();
            optimTrj.runShortCut( nbSteps, runId );
            optTime += optimTrj.getTime();
        }

        optimTrj.replaceP3dTraj();
        optimTrj.resetCostComputed();
        cout << "optimTrj.getRangeMax() : " << optimTrj.getRangeMax()  << endl;

        if( PlanEnv->getBool(PlanParam::trajComputeCostAfterPlannif) )
        {
            optimTrj.costStatistics( traj_statistics );

            cout << "--- stats on traj ---" << endl;
            cout << " length = " << traj_statistics.length << endl;
            cout << " max = " << traj_statistics.max << endl;
            cout << " average = " << traj_statistics.average << endl;
            cout << " integral = " << traj_statistics.integral << endl;
            cout << " mecha_work = " << traj_statistics.mecha_work << endl;
            cout << "---------------------" << endl;
        }

        traj = Move3D::Trajectory(optimTrj);
    }

    if( PlanEnv->getBool( PlanParam::withStomp ) )
    {
        //PlanEnv->setBool(PlanParam::withCurrentTraj,true);
        traj_optim_set_use_extern_trajectory( true );
        traj_optim_set_extern_trajectory( traj );

        traj_optim_runStompNoReset( runId );

        // traj = Move3D::Trajectory( rob, static_cast<p3d_rob*>(rob->getP3dRobotStruct())->tcur );
        traj.resetCostComputed();

        if( PlanEnv->getBool(PlanParam::trajComputeCostAfterPlannif) )
        {
            traj.costStatistics( traj_statistics );

            cout << "--- stats on traj ---" << endl;
            cout << " length = " << traj_statistics.length << endl;
            cout << " max = " << traj_statistics.max << endl;
            cout << " average = " << traj_statistics.average << endl;
            cout << " integral = " << traj_statistics.integral << endl;
            cout << " mecha_work = " << traj_statistics.mecha_work << endl;
            cout << "---------------------" << endl;
        }
    }
}

Move3D::Trajectory* move3d_run_rrt( Move3D::Robot* rob, Move3D::confPtr_t q_source, Move3D::confPtr_t q_target )
{
    rob->setInitPos( *q_source );
    rob->setGoalPos( *q_target );

    Move3D::Trajectory* path = move3d_planner_function( rob, q_source, q_target );

    if( path != NULL &&
            !PlanEnv->getBool(PlanParam::stopPlanner) &&
            PlanEnv->getBool(PlanParam::withSmoothing) )
    {
        double max_iteration = PlanEnv->getInt(PlanParam::smoothMaxIterations);
        double max_time = PlanEnv->getDouble( PlanParam::timeLimitSmoothing );

        move3d_smoothing_function( *path, max_iteration, max_time);
    }

    return path;
}

/**
 * Run Diffusion
 */
Move3D::Trajectory* or_runDiffusion( Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal )
{
    cout << "Run Diffusion" << endl;

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();
    if( robot == NULL )
    {
        cout << "robot not defined" << endl;
        return NULL;
    }

    Move3D::Trajectory* path = NULL;
//    try
    {
        p3d_SetStopValue(FALSE);

        timeval tim;
        gettimeofday(&tim, NULL);
        double t_init = tim.tv_sec+(tim.tv_usec/1000000.0);

        cout << "ENV.getBool(Env::Env::treePlannerIsEST) = " << ENV.getBool(Env::treePlannerIsEST) << endl;
        if (ENV.getBool(Env::treePlannerIsEST))
        {
            // move3d_run_est( robot );
            cout << "EST not implemented" << endl;
        }
        else
        {
            path = move3d_run_rrt( robot, q_init, q_goal );
        }

        gettimeofday(&tim, NULL);
        double dt = tim.tv_sec+(tim.tv_usec/1000000.0) - t_init;
        cout << "RRT computed in : " << dt << " sec" << endl;

        if( !ENV.getBool(Env::drawDisabled) ) {
            // g3d_draw_allwin_active();
        }
    }

    cout << " API_activeGraph : " << API_activeGraph << endl;

    if( API_activeGraph )
        API_activeGraph->draw();

//    catch (std::string str)
//    {
//        cerr << "Exeption in run qt_runDiffusion : " << endl;
//        cerr << str << endl;
//        ENV.setBool(Env::isRunning,false);
//    }
//    catch (std::runtime_error e)
//    {
//        cerr << "ERROR in move3d run rrt" << endl;
//    }
//    catch (...)
//    {
//        cerr << "Exeption in run qt_runDiffusion" << endl;
//        ENV.setBool(Env::isRunning,false);
//    }

    return path;
}

Move3D::Trajectory* or_runStomp( Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal  )
{
    cout << "Run Stomp" << endl;

    Move3D::Robot* robot = Move3D::global_Project->getActiveScene()->getActiveRobot();
    if( robot == NULL )
    {
        cout << "robot not defined" << endl;
        return false;
    }

    std::vector<Robot*> robots;
    robots.push_back( robot );

    // TODO see to remove this
    traj_optim_initScenario();

    std::vector<int> planner_joints = traj_optim_get_planner_joints();
    const CollisionSpace* coll_space = traj_optim_get_collision_space();
    std::vector<CollisionPoint> collision_points = traj_optim_get_collision_points();

    stomp_motion_planner::stompRun pool( coll_space, planner_joints, collision_points );
    pool.setPool( robots );
    // Uncomment for parallel stomps
    // pool.setRobotPool( 0, robots );

    Move3D::Trajectory T( robot );

//    if( init_stomp_.getNbOfViaPoints() == 0 )
    {
        T.push_back( q_init );
        T.push_back( q_goal );
    }
//    else{
//       T = init_stomp_;
//    }

    Move3D::Trajectory* path = NULL;

    if( pool.run( 0, T ) )
    {
        path = new Move3D::Trajectory( pool.getBestTrajectory(0) );
    }
    else
    {
        path = new Move3D::Trajectory( T );
    }

    return path;
}
