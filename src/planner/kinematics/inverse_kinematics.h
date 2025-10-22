#pragma once

#include "main/planner_headers.h"
#include "stl.h"
#include "str.h"
#include "utils.h"
#include <math.h>



class InverseKinematics {

public:
    // TODO: joint limits should also be deduced from urdf
    // TODO: introduce geometric features when kinematic engine exists

    /**
     * General function to compute inverse kinematics in an optimization-base manner for an abritrary robot.
     *
     * @param[out] optimized_values: Result of the optimization, input will be rewritten and can be arbitrary.
     * @param[in] urdf_path: Path to the urdf file.
     * @param[in] relevant_joint_names: Joints to consider for the optimization.
     * @param[in] initial_guess: Intial guess for the optimizer in joint space.
     * @param[in] upper_joint_limits: Max allowed values for joints specified in relevant_joint_names.
     * @param[in] lower_joint_limits: Min allowed values for joints specified in relevant_joint_names.
     * @param[in] cartesian_goal: Cartesian 7D-goal for the robot.
     * @param[in] cartesian_goal_cost: Costs per dimension associated with deviating from cartesian_goal. Values need to be between 0 and 1. 0 means that the corresponding dimension is not relevant for the optimization.
     */
    bool optimize(valarray<double>& optimized_values, const string& urdf_path, const std::vector<string>& relevant_joint_names, const valarray<double>& initial_guess, const valarray<double>& upper_joint_limits, const valarray<double>& lower_joint_limits, const valarray<double>& cartesian_goal, const valarray<double>& cartesian_goal_cost);

};
