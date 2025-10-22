#include "test_main.h"
#include "kinematics/inverse_kinematics.h"

struct GestaltPlannerTest       // TODO: use proper fk function when kinematic engine exists
{
    static auto& getState(GestaltPlanner& planner) { return planner.state; }
};

valarray<double> fw_kinematics_0(const valarray<double>& q, const string& urdf_path, const std::vector<string>& joint_names)   // TODO: use proper fk function when kinematic engine exists
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

// TODO: write some unit tests
TEST(test_planner, optimization_based_ik)
{
    // init
    string URDF_PATH = "../../../models/ur5e/ur5e.urdf";
    std::vector<string> JOINT_NAMES = { "shoulder_pan_joint",
                                       "shoulder_lift_joint",
                                       "elbow_joint",
                                       "wrist_1_joint",
                                       "wrist_2_joint",
                                       "wrist_3_joint" };
    GestaltPlanner planner("main_planner", true);
    planner.spawn("robot", URDF_PATH);
    planner.select_joints("robot", JOINT_NAMES);
    valarray<double> ones = { 1, 1, 1, 1, 1, 1 };
    valarray<double> q_start = -0.2 * ones;
    planner.set_joint_positions("robot", q_start);
    auto& state = *GestaltPlannerTest::getState(planner);
    valarray<double> upper_limits;
    valarray<double> lower_limits;
    state.getRobot("robot").getJointLimits(upper_limits, lower_limits);

    // define goal
    valarray<double> q_goal = 0.2 * ones; // this info is unavailable for the optimizer
    valarray<double> cartesian_goal = fw_kinematics_0(q_goal, URDF_PATH, JOINT_NAMES);
    valarray<double> cartesian_goal_cost = { 1.0,1.0,1.0,1.0,1.0,1.0,1.0 };
    makeDir("exports");
    planner.render_animation("start goal", "robot", { q_start, q_goal }, 1, "html", "exports/start_goal.html");

    // run IK
    InverseKinematics ik_calculator = InverseKinematics();
    valarray<double> optimized_values = {};
    bool success = ik_calculator.optimize(optimized_values, URDF_PATH, JOINT_NAMES, q_start, upper_limits, lower_limits, cartesian_goal, cartesian_goal_cost);
    std::cout << "success: " << success << std::endl;
    std::cout << "init values: " << q_start << std::endl;
    std::cout << "optimized values: " << optimized_values << std::endl;
    if (success) {
        planner.render_animation("start goal optimized", "robot", { q_start,  q_goal, optimized_values }, 1, "html", "exports/start_goal_optimized.html");
    }
}

TEST_MAIN

// times:
// whole optimization (initialization & one optim iteration): 3407 ms
// only fw kinematics execution during whole optimization: 3150 ms
// one fw kinematics call: 125-145 ms
