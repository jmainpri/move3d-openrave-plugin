#ifndef OPENRAVE_MOVE3D_API_HPP
#define OPENRAVE_MOVE3D_API_HPP

#include <openrave/openrave.h>

void move3d_set_or_api_environment_pointer( OpenRAVE::EnvironmentBasePtr env_ptr );
void move3d_draw_clear();

void move3d_set_or_api_scene();
void move3d_set_or_api_functions_configuration();
void move3d_set_or_api_functions_localpath();
void move3d_set_or_api_functions_robot();
void move3d_set_or_api_functions_joint();
void move3d_set_or_api_collision_space();
void move3d_set_or_api_functions_draw();

#endif // OPENRAVE_MOVE3D_API_HPP
