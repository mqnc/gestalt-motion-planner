
#include "planner_headers.h"


/*( public: )*/ bool GestaltPlanner::inverse_kinematics(
    valarray<double> optimized_values,
    string urdf_path,
    std::vector<string> relevant_joint_names,
    valarray<double> q_start,
    valarray<double> upper_limits,
    valarray<double> lower_limits,
    valarray<double> cartesian_goal,
    valarray<double> cartesian_goal_cost
) {
    InverseKinematics ik_calculator = InverseKinematics();
    bool success = ik_calculator.optimize(optimized_values, urdf_path, relevant_joint_names, q_start, upper_limits, lower_limits, cartesian_goal, cartesian_goal_cost);

    std::cout << "success: " << success << std::endl;
    std::cout << "init values: " << q_start << std::endl;
    std::cout << "optimized values: " << optimized_values << std::endl;

    return success;
    return true;
}
