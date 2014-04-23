#include "openrave_move3d_api.hpp"

#include "API/libmove3d_simple_api.hpp"

#include <libmove3d/planners/API/scene.hpp>
#include <libmove3d/planners/API/ConfigSpace/configuration.hpp>
#include <libmove3d/planners/API/ConfigSpace/localpath.hpp>
#include <libmove3d/planners/API/Device/robot.hpp>
#include <libmove3d/planners/API/Device/joint.hpp>
#include <libmove3d/planners/API/Trajectory/trajectory.hpp>
#include <libmove3d/planners/API/Graphic/drawModule.hpp>

#include <libmove3d/planners/planner/TrajectoryOptim/Stomp/stompOptimizer.hpp>

#define UNIX

#include <libmove3d/include/P3d-pkg.h>
#include <libmove3d/p3d/env.hpp>

#include <openrave/openrave.h>

#include <stdio.h>
#include <iostream>
#include <boost/bind.hpp>

using namespace std;
using namespace Eigen;
using namespace Move3D;

static OpenRAVE::EnvironmentBasePtr or_env_;

void move3d_set_or_api_environment_pointer( OpenRAVE::EnvironmentBasePtr env_ptr )
{
    or_env_ = env_ptr;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// SCENE
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_scene_constructor_fct( void* penv, std::string& name, std::vector<Robot*>& robots, std::string& active_robot )
{
    OpenRAVE::EnvironmentBase* scene = static_cast<OpenRAVE::EnvironmentBase*>( penv );
    std::vector< OpenRAVE::RobotBasePtr > or_robots;
    scene->GetRobots( or_robots );

    name = "";

    robots.clear();

    for( size_t i=0; i<or_robots.size(); i++ )
    {
        cout << "Add new Robot to Scene : " << or_robots[i]->GetName() << endl;

        robots.push_back( new Robot( or_robots[i].get() ) );
        robots.back()->setUseLibmove3dStruct( false );
    }

    cout << "All robots have been added !!! " << endl;

    if( !robots.empty() )
    {
        active_robot = robots[0]->getName();
        cout << "The Scene global_ActiveRobotName is : " << active_robot << endl;
    }
    else
    {
        active_robot = "";
        cout << "WARNING : the Scene global_ActiveRobotName has not been set!!!!" << endl;
    }

    return scene;
}

void move3d_scene_set_active_robot( Scene* sce, void* penv, const std::string& name )
{

}

Robot* move3d_scene_get_active_robot( Scene* sce, void* penv )
{
    cout << "sce : " << sce << endl;
    cout << "nb of robots : " << sce->getNumberOfRobots() << endl;
    return sce->getRobot(0);
}

double move3d_scene_get_dmax( void* penv )
{
    return ENV.getDouble(Env::dmax);
}

std::vector<double> move3d_scene_get_bounds( void* penv )
{
    vector<double> bounds(6);
    //    bounds[0] = scene->box.x1;
    //    bounds[1] = scene->box.x2;
    //    bounds[2] = scene->box.y1;
    //    bounds[3] = scene->box.y2;
    //    bounds[4] = scene->box.z1;
    //    bounds[5] = scene->box.z2;

    return bounds;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// ROBOT
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_robot_constructor( Robot* R, void* robotPt, unsigned int& nb_dofs, bool copy, std::string& name, std::vector<Joint*>& joints )
{
    OpenRAVE::RobotBase* robot = static_cast<OpenRAVE::RobotBase*>(robotPt);

    name = robot->GetName();

    cout << "constructing new robot : " << name << endl;

    joints.clear();

    for (size_t i=0; i<robot->GetJoints().size(); i++)
    {
        joints.push_back( new Joint( R, static_cast<OpenRAVE::KinBody::Joint*>(robot->GetJoints()[i].get()), i , copy ) );
    }

    nb_dofs = robot->GetDOF();

    return robot;
}

Move3D::Trajectory move3d_robot_get_current_trajectory(Robot* R)
{
    Move3D::Trajectory traj;
    return traj;
    //    Move3D::Trajectory traj( R, static_cast<p3d_rob*>(R->getP3dRobotStruct())->tcur );
    //    return traj;
}

confPtr_t move3d_robot_shoot( confPtr_t q, bool sample_passive )
{
    Robot* R = q->getRobot();
    // p3d_shoot( static_cast<p3d_rob*>(R->getP3dRobotStruct()), q->getConfigStruct(), sample_passive );

    size_t njnt = R->getNumberOfJoints(), i, j, k;
    double vmin, vmax;

    for( i=0; i<njnt; i++ )
    {
        Joint* jntPt = R->getJoint(i);

        for( j=0; j<jntPt->getNumberOfDof(); j++ )
        {
            k = jntPt->getIndexOfFirstDof() + j;

            if( true )
                // TODO
                //if (p3d_is_sampled_dof( robotPt, jntPt, j, sample_passive))
            {
                jntPt->getDofRandBounds( j, vmin, vmax );
                (*q)[k] = p3d_random( vmin, vmax );
            }
            else
            {
                (*q)[k] = jntPt->getJointDof( j );
            }
        }
    }

    return q;
}

bool move3d_robot_set_and_update( Robot* R, const Configuration& q, bool without_free_flyers )
{
    std::vector<OpenRAVE::dReal> config( R->getNumberOfDofs() );
    for( size_t i=0;i<config.size();i++) config[i] = q[i];
    static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct())->SetDOFValues( config );
    return true;
}

bool move3d_robot_set_and_update_multisol( Robot* R, const Configuration& q )
{
    return move3d_robot_set_and_update( R, q, false );
}

void move3d_robot_set_and_update_with_constraints( Robot* R, const Configuration& q )
{
    move3d_robot_set_and_update( R, q, false );
}

void move3d_robot_dealocate_void( OpenRAVE::RobotBase* )
{

}

bool move3d_robot_is_in_collision( Robot* R )
{
    OpenRAVE::RobotBasePtr robot = OpenRAVE::RobotBasePtr( static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct()), move3d_robot_dealocate_void );

    if( !robot->CheckSelfCollision() )
    {
        if( or_env_->CheckCollision(robot) )
            return true;
    }
    else {
        return true;
    }

    return false;
}

bool move3d_robot_is_in_collision_with_env( Robot* R )
{
    OpenRAVE::RobotBasePtr robot = OpenRAVE::RobotBasePtr( static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct()), move3d_robot_dealocate_void );
    return or_env_->CheckCollision(robot);
}

double move3d_robot_distance_to_env( Robot* R )
{
    return 0.0;
}

double move3d_robot_distance_to_robot( Robot* R1 , Robot* R2 )
{
    return 0.0;
}

confPtr_t move3d_robot_get_init_pos( Robot* R )
{
    return R->getNewConfig();
}

void move3d_robot_set_init_pos( Robot* R, const Configuration& q )
{

}

confPtr_t move3d_robot_get_goal_pos( Robot* R )
{
    return R->getNewConfig();
}

void move3d_robot_set_goal_pos( Robot* R, const Configuration& q )
{

}

confPtr_t move3d_robot_get_current_pos( Robot* R )
{
    OpenRAVE::RobotBase* robot = static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct());
    std::vector<OpenRAVE::dReal> q_tmp;
    robot->GetDOFValues( q_tmp );

    confPtr_t q = R->getNewConfig();

    for( size_t i=0;i<q_tmp.size();i++)
        (*q)[i] = q_tmp[i];

    return q;
}

confPtr_t move3d_robot_get_new_pos( Robot* R )
{
    return (confPtr_t (new Configuration( R, move3d_configuration_simple_constructor( R ), true )));
}

unsigned int move3d_robot_get_nb_active_joints( Robot* R )
{
    OpenRAVE::RobotBase* robot = static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct());
    return robot->GetActiveDOF();
}

Joint* move3d_robot_get_ith_active_joint( Robot* R,  unsigned int ithActiveDoF, unsigned int& ithDofOnJoint )
{
    unsigned int activeDof = 0;

    OpenRAVE::RobotBase* robot = static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct());
    const std::vector<Joint*>& joints = R->getJoints();
    const std::vector<int>& indices = robot->GetActiveDOFIndices();

    for(size_t i=0;i<joints.size();i++)
    {
        Joint* jntPt = joints[i];

        for(size_t j=0; j<jntPt->getNumberOfDof(); j++)
        {
            int k = jntPt->getIndexOfFirstDof() + j;

            if ( std::find(indices.begin(), indices.end(), k) != indices.end() )
            {
                if( activeDof == ithActiveDoF )
                {
                    ithDofOnJoint = j;
                    return joints[i];
                }

                activeDof++;
            }
        }
    }

    return NULL;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// JOINT
//
// ****************************************************************************************************
// ****************************************************************************************************

void* move3d_joint_constructor( Joint* J, void* jntPt, std::string& name )
{
    name = static_cast<OpenRAVE::KinBody::Joint*>(jntPt)->GetName();
    return static_cast<OpenRAVE::KinBody::Joint*>(jntPt);
}

Vector3d move3d_joint_get_vector_pos( const Joint* J )
{
    OpenRAVE::Vector p = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() )->GetSecondAttached()->GetTransform().trans;

    Vector3d v;
    v(0) = p[0];
    v(1) = p[1];
    v(2) = p[2];

    // cout << "anchor : " << v.transpose() << endl;

    return v;
}

Transform3d move3d_joint_get_matrix_pos( const Joint* J )
{
    OpenRAVE::KinBody::Joint* joint = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() );
    OpenRAVE::RaveTransformMatrix<double> T( joint->GetFirstAttached()->GetTransform() );
    OpenRAVE::Vector p = joint->GetAnchor();
    OpenRAVE::Vector right,up,dir,pos;
    T.Extract( right, up, dir, pos );

    Transform3d t;

    t(0,0) = right.x;
    t(1,0) = right.y;
    t(2,0) = right.z;
    t(3,0) = 0.0;

    t(0,1) = up.x;
    t(1,1) = up.y;
    t(2,1) = up.z;
    t(3,1) = 0.0;

    t(0,2) = dir.x;
    t(1,2) = dir.y;
    t(2,2) = dir.z;
    t(3,2) = 0.0;

    t(0,3) = p[0];
    t(1,3) = p[1];
    t(2,3) = p[2];
    t(3,3) = 1.0;

    return t;
}

void move3d_joint_shoot( Joint* J, Configuration& q, bool sample_passive )
{
    OpenRAVE::RobotBase* robot = static_cast<OpenRAVE::RobotBase*>( J->getRobot()->getRobotStruct() );

    const std::vector<int>& indices = robot->GetActiveDOFIndices();

    for( size_t j=0; j<J->getNumberOfDof(); j++)
    {
        int k = J->getIndexOfFirstDof() + j;

        if ( sample_passive || std::find(indices.begin(), indices.end(), k) != indices.end() )
        {
            double vmin,vmax;
            J->getDofRandBounds( j, vmin, vmax);
            q[k] = p3d_random( vmin, vmax );
        }
        else
        {
            q[k] = J->getJointDof( j );
        }
    }
}

double move3d_joint_get_joint_dof( const Joint* J, int ithDoF )
{
    OpenRAVE::KinBody::Joint* joint = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() );
    std::vector<OpenRAVE::dReal> values;
    joint->GetValues( values );
    return values[ithDoF];
}

void move3d_joint_set_joint_dof( const Joint* J, int ithDoF, double value )
{
    // p3d_jnt_set_dof( J->getP3dJointStruct(), ithDoF, value );
}

bool move3d_joint_is_joint_user( const Joint* J, int ithDoF )
{
    OpenRAVE::RobotBase* robot = static_cast<OpenRAVE::RobotBase*>( J->getRobot()->getRobotStruct() );

    // TODO see of that is ok
    int k = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() )->GetDOFIndex() + ithDoF;
    const std::vector<int>& indices = robot->GetActiveDOFIndices();
    return std::find( indices.begin(), indices.end(), k ) != indices.end();
}

void move3d_joint_get_joint_bounds( const Joint* J, int ithDoF, double& vmin, double& vmax )
{
    OpenRAVE::KinBody::Joint* joint = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() );
    std::vector<OpenRAVE::dReal> vLowerLimit;
    std::vector<OpenRAVE::dReal> vUpperLimit;
    joint->GetLimits( vLowerLimit, vUpperLimit );
    vmin = vLowerLimit[ithDoF];
    vmax = vUpperLimit[ithDoF];
}

void move3d_joint_get_joint_rand_bounds( const Joint* J, int ithDoF, double& vmin, double& vmax )
{
    move3d_joint_get_joint_bounds( J, ithDoF, vmin, vmax );
}

int move3d_joint_get_nb_of_dofs( const Joint* J )
{
    OpenRAVE::KinBody::Joint* joint = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() );
    return joint->GetDOF();
}

int move3d_joint_get_index_of_first_joint( const Joint* J )
{
    OpenRAVE::KinBody::Joint* joint = static_cast<OpenRAVE::KinBody::Joint*>( J->getJointStruct() );
    return joint->GetDOFIndex();
}

Joint* move3d_joint_get_previous_joint( const Joint* J, Robot* R )
{
    //    p3d_jnt* jntPt = (p3d_jnt*)(J->getP3dJointStruct());
    //    Joint* prevJnt=NULL; int found=0;

    //    for (unsigned int i=0; i<R->getNumberOfJoints(); i++ )
    //    {
    //        Joint* jnt = R->getJoint(i);

    //        if ( jntPt->prev_jnt == jnt->getP3dJointStruct() )
    //        {
    //            found++;
    //            prevJnt = jnt;
    //        }
    //    }

    //    if (found == 1) {
    //        return prevJnt;
    //    }
    //    else if (found > 1 ) {
    //        cout << "Found : " << found << " prev. joints!!!" << endl;
    //    }

    return NULL;
}

double move3d_joint_get_dist( const Joint* J )
{
    // return J->getP3dJointStruct()->dist;
    return 0.1;
}

double move3d_joint_is_joint_dof_angular( const Joint* J, int ithDoF )
{
    return false;
}

double move3d_joint_is_joint_dof_circular( const Joint* J, int ithDoF )
{
    return false;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// COLLISION SPACE
//
// ****************************************************************************************************
// ****************************************************************************************************

int move3d_get_nb_collision_points()
{
    cout << __PRETTY_FUNCTION__ << endl;

    std::vector<OpenRAVE::RobotBasePtr> robots;

    or_env_->GetRobots( robots );

    if( robots.empty() ) {
        RAVELOG_ERROR( "No robots in environment \n" );
        return 0;
    }

    OpenRAVE::CollisionReportPtr report( new OpenRAVE::CollisionReport() );
    or_env_->CheckCollision( robots[0], report );

    return report->contacts.size();
}

bool move3d_get_config_collision_cost( Move3D::Robot* R, int i, Eigen::MatrixXd& collision_point_potential, std::vector< std::vector<Eigen::Vector3d> >& collision_point_pos )
{
    // cout << __PRETTY_FUNCTION__ << endl;

    OpenRAVE::RobotBasePtr robot = OpenRAVE::RobotBasePtr( static_cast<OpenRAVE::RobotBase*>(R->getRobotStruct()), move3d_robot_dealocate_void );
    OpenRAVE::CollisionReportPtr report( new OpenRAVE::CollisionReport() );

    bool in_collision = or_env_->CheckCollision( robot, report );

    Eigen::Vector3d p;

    // calculate the position of every collision point
    for (size_t j=0; j<report->contacts.size(); j++)
    {
        collision_point_potential(i,j) = report->contacts[j].depth;

        p(0) = report->contacts[j].pos.x;
        p(1) = report->contacts[j].pos.y;
        p(2) = report->contacts[j].pos.z;

        collision_point_pos[i][j] = p;

        // if ( colliding )
        // {
            // This is the function that discards joints too close to the base
            // if( planning_group_->collision_points_[j].getSegmentNumber() > 1 )
            //{
                // in_collision = true;
            //}
        // }
    }

    return in_collision;
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// DAW
//
// ****************************************************************************************************
// ****************************************************************************************************

std::vector< boost::shared_ptr<void> > graphptr;

void move3d_draw_clear()
{
    graphptr.clear();
}

void move3d_draw_sphere_fct( double x, double y, double z, double radius )
{
    std::vector<OpenRAVE::RaveVector<float> > vpoints;
    OpenRAVE::RaveVector<float> pnt(x,y,z);
    vpoints.push_back(pnt);

    std::vector<float> vcolors;
    vcolors.push_back(0);
    vcolors.push_back(0);
    vcolors.push_back(1);

    OpenRAVE::GraphHandlePtr fig = or_env_->plot3( &vpoints[0].x, vpoints.size(), sizeof(vpoints[0]), 10, &vcolors[0], 1 );

    graphptr.push_back( fig );

    // cout << "Add sphere : " << x << " , " << y << " , " << z << endl;
    // g3d_draw_solid_sphere( x, y, z, radius, 10 );
}

void move3d_draw_one_line_fct( double x1, double y1, double z1, double x2, double y2, double z2, int color, double *color_vect )
{
    int nb_points = 2;
    float* ppoints = new float[3*nb_points];

    int i=0;
    ppoints[i+0] = x1;
    ppoints[i+1] = y1;
    ppoints[i+2] = z1;

    i += 3;
    ppoints[i+0] = x2;
    ppoints[i+1] = y2;
    ppoints[i+2] = z2;

    float colors[] = {0,0,0};
    if( color_vect != NULL ) {
        colors[0] = color_vect[0];
        colors[1] = color_vect[1];
        colors[2] = color_vect[2];
    }

    OpenRAVE::GraphHandlePtr fig = or_env_->drawlinelist( ppoints, nb_points, 3*sizeof(float), 3.0, colors );

    delete ppoints;

    graphptr.push_back( fig );

    // cout << "Add line : " << x1 << " , " << y1 << " , " << z1 << endl;
    // g3d_drawOneLine( x1, y1, z1, x2, y2, z2, color, color_vect );
}

// ****************************************************************************************************
// ****************************************************************************************************
//
// SETTER
//
// ****************************************************************************************************
// ****************************************************************************************************

void move3d_set_or_api_scene()
{
    move3d_set_api_functions( false );
    move3d_set_fct_scene_constructor( boost::bind( move3d_scene_constructor_fct, _1, _2, _3, _4 ) );
    move3d_set_fct_set_active_robot( boost::bind( move3d_scene_set_active_robot, _1, _2, _3 ) );
    move3d_set_fct_get_active_robot( boost::bind( move3d_scene_get_active_robot, _1, _2 ) );
    move3d_set_fct_get_dmax( boost::bind( move3d_scene_get_dmax, _1 ) );
    move3d_set_fct_get_bounds( boost::bind( move3d_scene_get_bounds, _1 ) );
}

void move3d_set_or_api_functions_configuration()
{
    move3d_set_api_functions_configuration_simple();
}

void move3d_set_or_api_functions_localpath()
{
    move3d_set_api_functions_localpath_simple();
}

void move3d_set_or_api_functions_robot()
{
    move3d_set_fct_robot_constructor( boost::bind( move3d_robot_constructor, _1, _2, _3, _4, _5, _6 ) );
    move3d_set_fct_robot_get_current_trajectory( boost::bind( move3d_robot_get_current_trajectory, _1 ) );
    move3d_set_fct_robot_shoot( boost::bind( move3d_robot_shoot, _1, _2 ) );
    // move3d_set_fct_robot_shoot_dir( boost::bind( move3d_robot_set_and_update, _1, _2, _3 ));
    move3d_set_fct_robot_set_and_update( boost::bind( move3d_robot_set_and_update, _1, _2, _3 ));
    move3d_set_fct_robot_set_and_update_multi_sol( boost::bind( move3d_robot_set_and_update_multisol, _1 , _2 ) );
    move3d_set_fct_robot_without_constraints( boost::bind( move3d_robot_set_and_update_with_constraints, _1, _2 ) );
    move3d_set_fct_robot_is_in_collision( boost::bind( move3d_robot_is_in_collision, _1 ));
    move3d_set_fct_robot_is_in_collision_with_others_and_env( boost::bind( move3d_robot_is_in_collision_with_env, _1 ) );
    move3d_set_fct_robot_distance_to_env( boost::bind( move3d_robot_distance_to_env, _1 ) ) ;
    move3d_set_fct_robot_distance_to_robot( boost::bind( move3d_robot_distance_to_robot, _1, _2 ) );
    move3d_set_fct_robot_get_init_pos( boost::bind( move3d_robot_get_init_pos, _1 ) );
    move3d_set_fct_robot_set_init_pos( boost::bind( move3d_robot_set_init_pos, _1, _2 ) );
    move3d_set_fct_robot_get_goal_pos( boost::bind( move3d_robot_get_goal_pos, _1 ) );
    move3d_set_fct_robot_set_goal_pos( boost::bind( move3d_robot_set_goal_pos, _1, _2 ) );
    move3d_set_fct_robot_get_current_pos( boost::bind( move3d_robot_get_current_pos, _1 ) );
    move3d_set_fct_robot_get_new_pos( boost::bind( move3d_robot_get_new_pos, _1 ) );
    move3d_set_fct_robot_get_number_of_active_dofs( boost::bind( move3d_robot_get_nb_active_joints, _1 ) );
    move3d_set_fct_robot_get_ith_active_dof_joint( boost::bind( move3d_robot_get_ith_active_joint, _1, _2 , _3 ) );
}

void move3d_set_or_api_functions_joint()
{
    move3d_set_fct_joint_constructor( boost::bind( move3d_joint_constructor, _1, _2, _3 ) );
    move3d_set_fct_joint_get_vector_pos( boost::bind( move3d_joint_get_vector_pos, _1 ) );
    move3d_set_fct_joint_get_matrix_pos( boost::bind( move3d_joint_get_matrix_pos, _1 ) );
    move3d_set_fct_joint_joint_shoot( boost::bind( move3d_joint_shoot, _1, _2, _3 ) );
    move3d_set_fct_joint_get_joint_dof( boost::bind( move3d_joint_get_joint_dof, _1, _2 ) );
    move3d_set_fct_joint_is_joint_dof_angular( boost::bind( move3d_joint_is_joint_dof_angular, _1, _2 ) );
    move3d_set_fct_joint_is_joint_dof_circular( boost::bind( move3d_joint_is_joint_dof_circular, _1, _2 ) );
    move3d_set_fct_joint_set_joint_dof( boost::bind( move3d_joint_set_joint_dof, _1, _2, _3 ) );
    move3d_set_fct_joint_is_joint_user( boost::bind( move3d_joint_is_joint_user, _1, _2 ) );
    move3d_set_fct_joint_get_bound( boost::bind( move3d_joint_get_joint_bounds, _1, _2, _3, _4 ) );
    move3d_set_fct_joint_get_bound_rand( boost::bind( move3d_joint_get_joint_rand_bounds, _1, _2, _3, _4 ) );
    move3d_set_fct_joint_get_nb_of_dofs( boost::bind( move3d_joint_get_nb_of_dofs, _1 ) );
    move3d_set_fct_joint_get_index_of_first_dof( boost::bind( move3d_joint_get_index_of_first_joint, _1 ) );
    move3d_set_fct_joint_get_previous_joint( boost::bind( move3d_joint_get_previous_joint, _1, _2 ) );
    move3d_set_fct_joint_joint_dist( boost::bind( move3d_joint_get_dist, _1 ) );
}

void move3d_set_or_api_collision_space()
{
    move3d_set_api_functions_collision_space( false );
    move3d_set_fct_get_nb_collision_points( boost::bind( move3d_get_nb_collision_points ) );
    move3d_set_fct_get_config_collision_cost( boost::bind( move3d_get_config_collision_cost, _1, _2, _3, _4 ) ) ;
}

void move3d_set_or_api_functions_draw()
{
    move3d_set_fct_draw_sphere( boost::bind( move3d_draw_sphere_fct, _1, _2, _3, _4 ) );
    move3d_set_fct_draw_one_line( boost::bind( move3d_draw_one_line_fct, _1, _2, _3, _4, _5, _6, _7, _8 ) );
    move3d_set_fct_draw_clear_handles( boost::bind( move3d_draw_clear ) );
}

