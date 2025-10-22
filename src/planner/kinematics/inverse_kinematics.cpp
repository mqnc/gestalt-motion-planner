#include "inverse_kinematics.h"
#include "ceres/ceres.h"

struct GestaltPlannerTest       // TODO: use proper fk function when kinematic engine exists
{
    static auto& getState(GestaltPlanner& planner) { return planner.state; }
};

valarray<double> fw_kinematics_1(const valarray<double>& q, const string& urdf_path, const std::vector<string>& joint_names)   // TODO: use proper fk function when kinematic engine exists
{
    // init fw_kin_planner, ugly
    GestaltPlanner planner("fw_kinematics_planner", false);
    auto& state = *GestaltPlannerTest::getState(planner);
    planner.spawn("fw_kinematics_robot", urdf_path);
    planner.select_joints("fw_kinematics_robot", joint_names);

    state.getRobot("fw_kinematics_robot").setJointPositions(q);
    auto pose = state.getRobot("fw_kinematics_robot").getPartByLinkId("ee_link").bulletObject.getWorldTransform(); // assumes that "ee_link" is a convention for TCP
    valarray<double> y = { double(pose.getOrigin().x()),
                          double(pose.getOrigin().y()),
                          double(pose.getOrigin().z()),
                           double(pose.getRotation().w()),
                           double(pose.getRotation().x()),
                           double(pose.getRotation().y()),
                           double(pose.getRotation().z())
    };
    return y;
};


bool InverseKinematics::optimize(valarray<double>& optimized_values, const string& urdf_path, const std::vector<string>& relevant_joint_names, const valarray<double>& initial_guess, const valarray<double>& upper_joint_limits, const valarray<double>& lower_joint_limits, const valarray<double>& cartesian_goal, const valarray<double>& cartesian_goal_cost)
{
    struct distance
    {
        distance(const valarray<double>& cartesian_goal, const valarray<double>& cartesian_goal_cost, const string& urdf_path, const std::vector<string>& joint_names):
            cartesian_goal_(cartesian_goal), cartesian_goal_cost_(cartesian_goal_cost), urdf_path_(urdf_path), joint_names_(joint_names) {}

        bool operator()(double const* const* parameters, double* residuals) const
        {
            // fk with parameters
            valarray<double> q;
            q.resize(size(joint_names_));
            for (int i = 0; i < size(joint_names_); ++i) {
                q[i] = parameters[0][i];
            }
            // std::cout << "-----------------------" << std::endl;
            // std::cout << q << std::endl;
            valarray<double> tcp_position = fw_kinematics_1(q, urdf_path_, joint_names_);

            // computation of cost
            double t0 = cartesian_goal_cost_[0] * pow((tcp_position[0] - cartesian_goal_[0]), 2.0);
            double t1 = cartesian_goal_cost_[1] * pow((tcp_position[1] - cartesian_goal_[1]), 2.0);
            double t2 = cartesian_goal_cost_[2] * pow((tcp_position[2] - cartesian_goal_[2]), 2.0);
            double t3 = cartesian_goal_cost_[3] * pow((tcp_position[3] - cartesian_goal_[3]), 2.0);
            double t4 = cartesian_goal_cost_[4] * pow((tcp_position[4] - cartesian_goal_[4]), 2.0);
            double t5 = cartesian_goal_cost_[5] * pow((tcp_position[5] - cartesian_goal_[5]), 2.0);
            double t6 = cartesian_goal_cost_[6] * pow((tcp_position[6] - cartesian_goal_[6]), 2.0);
            double distance = sqrt(t0 + t1 + t2 + t3 + t4 + t5 + t6);

            residuals[0] = distance;

            return true;
        }

        valarray<double> cartesian_goal_;
        valarray<double> cartesian_goal_cost_;
        string urdf_path_;
        std::vector<string> joint_names_;
    };

    // sanity checks
    if ((size(cartesian_goal) != 7) || (size(cartesian_goal_cost) != 7)) {
        std::cout << "goal inputs are not in cartesian space!" << std::endl;
        std::cout << "size cartesian_goal: " << size(cartesian_goal) << std::endl;
        std::cout << "size cartesian_goal_cost: " << size(cartesian_goal_cost) << std::endl;
        return false;
    }
    for (double elem : cartesian_goal_cost) {
        if (((elem) > 1) || ((elem) < 0)) {
            std::cout << "cartesian_goal_cost is not between 0 and 1!" << std::endl;
            return false;
        }
    }
    int dimensions_joint_space = size(relevant_joint_names);
    if ((size(initial_guess) != dimensions_joint_space) || (size(upper_joint_limits) != dimensions_joint_space) || (size(lower_joint_limits) != dimensions_joint_space)) {
        std::cout << "dimensions for the joint_space are not consistent for relevant_joint_names, initial_guess, upper_joint_limits & lower_joint_limits!" << std::endl;
        return false;
    }


    std::cout << "solving IK problem with ceres" << std::endl;

    // Defining the problem
    ceres::Problem problem;
    double* variables = new double[size(initial_guess)];
    for (int i = 0; i < size(initial_guess); ++i) {
        variables[i] = initial_guess[i];
    }
    // example for dynamic cost-function here: https://github.com/ceres-solver/ceres-solver/blob/master/examples/robot_pose_mle.cc
    ceres::DynamicNumericDiffCostFunction<distance>* cost_function = new ceres::DynamicNumericDiffCostFunction<distance>(new distance(cartesian_goal, cartesian_goal_cost, urdf_path, relevant_joint_names));
    cost_function->AddParameterBlock(size(initial_guess));
    cost_function->SetNumResiduals(1);
    problem.AddResidualBlock(cost_function, nullptr, variables);
    for (int i = 0; i < size(initial_guess); ++i) {
        problem.SetParameterUpperBound(variables, i, upper_joint_limits[i] - 0.1);
        problem.SetParameterLowerBound(variables, i, lower_joint_limits[i] + 0.1);
    }


    // // Checking definitions
    // std::cout << "upper_joint_limits" << upper_joint_limits << std::endl;
    // std::cout << "lower_joint_limits" << lower_joint_limits << std::endl;
    // for (int i = 0; i < size(initial_guess); ++i) {
    //     std::cout << "loopstep: " << i << std::endl;
    //     std::cout << "initial value: " << initial_guess[i] << std::endl;
    //     std::cout << "joint name: " << relevant_joint_names[i] << std::endl;
    //     std::cout << "upper bound in ceres: " << problem.GetParameterUpperBound(variables, i) << std::endl;
    //     std::cout << "lower bound in ceres: " << problem.GetParameterLowerBound(variables, i) << std::endl;
    //     std::cout << "---------------------------------------" << std::endl;
    // }

    // Run the solver
    ceres::Solver::Options options;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 10;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // Pass the solution
    optimized_values.resize(size(relevant_joint_names));
    for (int i = 0; i < size(relevant_joint_names); ++i) {
        optimized_values[i] = variables[i];
    }

    return true;
};
