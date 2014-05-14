#ifndef PLANNER_FUNCTIONS_HPP
#define PLANNER_FUNCTIONS_HPP

#include <libmove3d/planners/API/ConfigSpace/configuration.hpp>
#include <libmove3d/planners/API/Trajectory/trajectory.hpp>

std::vector<Move3D::Trajectory*> or_runDiffusion( Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal  );
std::vector<Move3D::Trajectory*> or_runStomp( Move3D::confPtr_t q_init, Move3D::confPtr_t q_goal  );

#endif // PLANNER_FUNCTIONS_HPP
