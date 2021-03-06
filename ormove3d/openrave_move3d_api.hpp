#ifndef OPENRAVE_MOVE3D_API_HPP
#define OPENRAVE_MOVE3D_API_HPP

#include <openrave/openrave.h>

void move3d_set_or_api_environment_pointer( OpenRAVE::EnvironmentBasePtr env_ptr );
void move3d_set_or_api_environment_clones_pointer( std::vector<OpenRAVE::EnvironmentBasePtr> env_clones );
void move3d_or_api_environment_clones_clear();
void move3d_or_api_add_handles();
void move3d_or_api_clear_all_handles();
void move3d_or_set_robot_is_puck(bool is_puck);

void move3d_set_or_api_scene();
void move3d_set_or_api_functions_configuration();
void move3d_set_or_api_functions_localpath();
void move3d_set_or_api_functions_robot();
void move3d_set_or_api_functions_joint();
void move3d_set_or_api_collision_space();
void move3d_set_or_api_functions_draw();

#endif // OPENRAVE_MOVE3D_API_HPP
