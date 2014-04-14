#ifndef PLANNER_FUNCTIONS_HPP
#define PLANNER_FUNCTIONS_HPP

#include <libmove3d/planners/API/ConfigSpace/configuration.hpp>

bool or_runDiffusion( Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal  );
bool or_runStomp( Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal  );

#endif // PLANNER_FUNCTIONS_HPP
