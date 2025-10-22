
#include "stl.h"
#include "str.h"
#include "utils.h"

#include "test_main.h"

#include "btTools.h"
#include "btBulletCollisionCommon.h"
#include "main/planner_headers.h"

// #define DIFFICULT_PATH_PLANNING_TEST
// #define MANY_INTERPOLATION_TESTS

struct GestaltPlannerTest {
	static auto& getState(GestaltPlanner& planner) { return planner.state; }
};

struct CollisionRobotTest {
	static auto& getParent(CollisionRobot& robot) { return robot.parent; }
	static auto& getChildren(CollisionRobot& robot) { return robot.children; }
	static auto& getRootJoint(CollisionRobot& robot) { return robot.root; }
	static auto& getJointMap(CollisionRobot& robot) { return robot.jointMap; }
};

#define THIS_TEST_LOG_NAME \
(string(::testing::UnitTest::GetInstance()->current_test_info()->name()) + ".log.cpp")
	; // for formatter

#ifdef NAY
TEST(test_planner, construction) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);
		EXPECT_EQ(state.robots.size(), 1);
		EXPECT_EQ(state.robots.count("__world__"), 1);

		auto groups = state.collisionIgnoreGroupManager.getGroups();
		EXPECT_EQ(groups.size(), 1);
		EXPECT_EQ(groups.count("__passive__"), 1);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, cache_file) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.cache_file(
			"does_not_exist.urdf",
			R"(
		<?xml version="1.0"?>
		<robot name="world">
			<link name="origin">
			</link>
		</robot>
		)"
		);
		planner.spawn("myThing", "does_not_exist.urdf");
		EXPECT_EQ(state.robots.count("myThing"), 1);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, spawn__remove__reset) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.spawn(
			"o1",
			"../../../src/test/urdf/basics.urdf",
			"",
			Pose {10}
		);
		planner.spawn(
			"o2",
			"../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml",
			Pose {1, 2, 3, 0, 0, 1, 0},
			"static",
			{-0.1, -0.2, -0.3, -0.4, -0.5, -0.6},
			"o1",
			"sphere"
		);

		EXPECT_FLOAT_EQ(
			state.getRobot("o2").getPartTrafoInWorld().getOrigin().x(), 11);

		planner.remove("o1"); // should remove the child o2 as well

		auto groups = state.collisionIgnoreGroupManager.getGroups();
		EXPECT_EQ(groups.size(), 1);
		EXPECT_EQ(groups.count("__passive__"), 1);

		planner.spawn("o3", "../../../src/test/urdf/basics.urdf");

		planner.reset();
		EXPECT_EQ(groups.size(), 1);
		EXPECT_EQ(groups.count("__passive__"), 1);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, export_state) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.spawn(
			"o1",
			"../../../src/test/urdf/basics.urdf",
			"",
			Pose {10}
		);
		planner.spawn(
			"o2",
			"../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml",
			Pose {1, 2, 3, 0, 0, 1, 0},
			"static",
			{-0.1, -0.2, -0.3, -0.4, -0.5, -0.6},
			"o1",
			"sphere"
		);

		auto output = planner.export_state();
		EXPECT_EQ(std::hash<std::string> {}(output), 11710879826062368227ul)
			<< " if this test fails because the export has changed, manually inspect the new export and hard code its new hash here.\n\n" << output;

		output = planner.export_state("o1", false);
		EXPECT_EQ(std::hash<std::string> {}(output), 4480886955724339022ul)
			<< " if this test fails because the export has changed, manually inspect the new export and hard code its new hash here.\n\n" << output;


	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, set_base_pose) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.spawn("o1", "../../../src/test/urdf/basics.urdf", "",
			Pose {1, 2, 3, 0, 0, 1, 0});
		planner.spawn("o2", "../../../src/test/urdf/basics.urdf", "",
			Pose {1, 2, 3, 0, 0, 1, 0});

		planner.set_base_pose("o1", PoseUpdate {4, 5});

		auto pose = state.getRobot("o1").getPartTrafoInWorld();
		EXPECT_FLOAT_EQ(pose.getOrigin().x(), 4);
		EXPECT_FLOAT_EQ(pose.getOrigin().z(), 3);
		EXPECT_FLOAT_EQ(pose.getRotation().z(), 1);

		planner.set_base_pose("o1", Pose {1, 2, 3, 0, 0, 1, 0},
			"o2", "box");

		pose = state.getRobot("o1").getPartTrafoInWorld();
		EXPECT_FLOAT_EQ(pose.getOrigin().x(), 0);
		EXPECT_FLOAT_EQ(pose.getOrigin().y(), 0);
		EXPECT_FLOAT_EQ(pose.getOrigin().z(), 6);
		EXPECT_FLOAT_EQ(pose.getRotation().w(), 1);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}
#endif
TEST(test_planner, set_joint_positions) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		// no specified joints, all urdf joints should be selected
		planner.spawn(
			"o1",
			"../../../models/ur5e/ur5e.urdf"
		);
		EXPECT_EQ(planner.get_joint_selection("o1"), (vector<string> {
				"shoulder_pan_joint",
				"shoulder_lift_joint",
				"elbow_joint",
				"wrist_1_joint",
				"wrist_2_joint",
				"wrist_3_joint"
		}));
		planner.select_joints("o1", {"shoulder_pan_joint", "shoulder_lift_joint"});
		EXPECT_EQ(CollisionRobotTest::getJointMap(
					  state.getRobot("o1")) .size(), 2);
		planner.select_joints("o1"); // restore default joint selection
		EXPECT_EQ(CollisionRobotTest::getJointMap(
					  state.getRobot("o1")) .size(), 6);

		// see if shuffled joint names are handled correctly
		planner.set_joint_positions("o1", {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
		planner.select_joints("o1", {
			"wrist_3_joint",
			"wrist_2_joint",
			"wrist_1_joint",
			"elbow_joint",
			"shoulder_lift_joint",
			"shoulder_pan_joint"
		});
		// .min() == 1 for all true
		EXPECT_TRUE(
			(state.getRobot("o1").getJointPositions() == valarray<double> {0.6, 0.5, 0.4, 0.3, 0.2, 0.1}).min() == 1
		);
		planner.set_joint_positions("o1", {0.1, 0.2, 0.3, 0.4, 0.5, 0.6});
		planner.select_joints("o1"); // restore default joint selection
		// .min() == 1 for all true
		EXPECT_TRUE(
			(state.getRobot("o1").getJointPositions() == valarray<double> {0.6, 0.5, 0.4, 0.3, 0.2, 0.1}).min() == 1
		);

		// joints specified in yaml file
		planner.spawn(
			"o2",
			"../../../models/ur5e/ur5e.urdf",
			"../../../src/test/urdf/ur5e_less_joints.yaml"
		);
		EXPECT_EQ(CollisionRobotTest::getJointMap(
					  state.getRobot("o2")) .size(), 3);

		// manually selected joints
		planner.spawn(
			"o3",
			"../../../models/ur5e/ur5e.urdf",
			"../../../src/test/urdf/ur5e_less_joints.yaml",
			{}, "static"
		);
		planner.select_joints("o3", {"wrist_1_joint", "wrist_2_joint"});
		EXPECT_EQ(CollisionRobotTest::getJointMap(
					  state.getRobot("o3")) .size(), 2);
		planner.select_joints("o3");
		EXPECT_EQ(CollisionRobotTest::getJointMap(
					  state.getRobot("o3")) .size(), 3);

		// actuate one of the specified joints
		auto before1 = state.getRobot("o1") .getPartByLinkId("ee_link")
						   .bulletObject.getWorldTransform();

		planner.set_joint_positions("o1",
			{unchangedNaN, unchangedNaN, unchangedNaN,
				unchangedNaN, unchangedNaN, 1});
		auto after1 = state.getRobot("o1") .getPartByLinkId("ee_link")
						  .bulletObject.getWorldTransform();
		EXPECT_FALSE(before1 == after1);


		GestaltPlanner planner2(THIS_TEST_LOG_NAME);
		valarray<double> ones = {1, 1, 1, 1, 1, 1};
		planner2.spawn(
			"robot", "../../../models/ur5e/ur5e.urdf"
		);

		planner2.select_joints("robot", {
				"shoulder_pan_joint",
				"shoulder_lift_joint",
				"elbow_joint",
				"wrist_1_joint",
				"wrist_2_joint",
				"wrist_3_joint"});

		makeDir("exports");
		planner2.render_animation(
			"joint test", "robot",
			{-0.1 * ones, 0.1 * ones}, 1,
			"html", "exports/joint_test.html");

		planner2.render_animation(
			"elbow up", "robot",
			{
				{1, -1, 1.5, 0.1, 0.2, 0.3},
				{1, -0.924339, 1.94446, 2.72147, -0.2, -2.84159},
				{-1.6374, -2.3014, -1.8275, 0.807477, 2.46371, -2.39161},
				{-1.6374, -2.04437, -1.61121, -2.80745, -2.46371, 0.749982}
			}, 1,
			"html", "exports/elbow_up.html");

		planner2.render_animation(
			"elbow down", "robot",
			{
				{1, 0.425252, -1.5, 1.67475, 0.2, 0.3},
				{1, 0.902561, -1.94446, -1.49969, -0.2, -2.84159},
				{-1.6374, 2.25825, 1.8275, -1.124, 2.46371, -2.39161},
				{-1.6374, 2.71114, 1.61121, 1.781, -2.46371, 0.749982}
			}, 1,
			"html", "exports/elbow_down.html");
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}
#ifdef NAY
TEST(test_planner, find_collisions__collision_ignore_groups) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		// no collision ignore groups
		planner.spawn("o1", "../../../models/ur5e/ur5e.urdf");
		EXPECT_EQ(planner.find_collisions().size(), 0);

		// ur5 without collision groups collides with margin
		planner.set_safety_margin("o1", 0.01);
		EXPECT_GT(planner.find_collisions().size(), 0);
		// ignore all collisions for o1
		planner.create_collision_ignore_group("myGroup", {"o1"});
		EXPECT_EQ(planner.find_collisions().size(), 0);
		planner.create_collision_ignore_group("myOtherGroup", {"myGroup"});
		planner.delete_collision_ignore_group("myGroup");
		EXPECT_EQ(planner.find_collisions().size(), 0);

		planner.remove("o1");
		// only __passive__ group should be left
		EXPECT_EQ(state.collisionIgnoreGroupManager.getGroups().size(), 1);

		// collision ignore groups as specified in yaml
		planner.spawn("o1", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml");
		EXPECT_EQ(planner.find_collisions().size(), 0);
		planner.set_safety_margin("o1", 0.01);
		EXPECT_EQ(planner.find_collisions().size(), 0);
		planner.delete_collision_ignore_group("o1.ignore_base_vs_upper_arm");
		EXPECT_EQ(planner.find_collisions().size(), 1);
		planner.create_collision_ignore_group("test",
			{"o1.base_link", "o1.upper_arm_link"});
		EXPECT_EQ(planner.find_collisions().size(), 0);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, set_safety_margin) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.cache_file(
			"box.urdf",
			R"(
		<?xml version="1.0"?>
		<robot name="box">
			<link name="box">
				<collision>
					<geometry>
						<box size="2 2 2" />
					</geometry>
				</collision>
			</link>
		</robot>
		)"
		);
		planner.spawn("box1", "box.urdf");
		planner.spawn("box2", "box.urdf", "", Pose {3, 3});

		EXPECT_EQ(planner.find_collisions().size(), 0);
		planner.set_safety_margin("*", 0.51);
#ifdef COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
		EXPECT_EQ(planner.find_collisions().size(), 0);
		planner.render_scene("box test", "__world__", "html", "exports/rounded_box.html");
#else
		EXPECT_GT(planner.find_collisions().size(), 0);
		planner.render_scene("box test", "__world__", "html", "exports/sharp_box.html");
#endif
		planner.set_safety_margin("*", 0.71);
		EXPECT_GT(planner.find_collisions().size(), 0);
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}


// TEST(test_planner, export__import) {
// 	GestaltPlanner planner(THIS_TEST_LOG_NAME);
// 	auto& state = *GestaltPlannerTest::getState(planner);

// 	planner.spawn(
// 		"o1",
// 		"../../../src/test/urdf/wrist3.urdf",
// 		"",
// 		Pose{ 10 }
// 	);
// 	planner.set_safety_margin("*", 0.012);

// 	planner.export_state("exports/export_minimal_test.json");
// 	planner.export_state("exports/export_test.msgpack", "msgpack");
// 	GestaltPlanner planner2(THIS_TEST_LOG_NAME);
// 	planner2.import_state("exports/export_minimal_test.json");

// 	auto json1 = planner.render_scene("json test", "o1", "json", "basicscene1.json");
// 	auto json2 = planner2.render_scene("json test", "o1", "json", "basicscene2.json");

// 	cout << json::diff(json::parse(json1), json::parse(json2)) << "\n";

// }

TEST(test_planner, param_structs__json_call) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);

		planner.json_call(R"({
		"jsonrpc":"2.0",
		"method":"spawn",
		"params":{
			"object_id":"o1",
			"description_file":"../../../src/test/urdf/wrist3.urdf"
		},
		"id":1
	})");
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, plan_smoothen_export_import) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		// planner.cache_file(
		// 	"floor.urdf",
		// 	R"(<?xml version="1.0"?><robot name="floor"><link name="floor">
		// 	<collision><origin xyz="0 0 -0.02" rpy="0 0 0" />
		// 	<geometry><cylinder radius="1" length="0.04" /></geometry>
		// 	</collision></link></robot>)");

		planner.cache_file(
			"floor.urdf",
			R"(<?xml version="1.0"?><robot name="floor"><link name="floor">
		<collision><origin xyz="0 0 -0.02" rpy="0 0 0" />
		<geometry><box size="1 1 0.04" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"sword.urdf",
			R"(<?xml version="1.0"?><robot name="sword"><link name="sword">
		<collision><origin xyz="0.02 0 -0.28" rpy="0 0 0" />
		<geometry><cylinder radius="0.02" length="0.6" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"pillar.urdf",
			R"(<?xml version="1.0"?><robot name="pillar"><link name="pillar">
		<collision><origin xyz="0 0 0.5" rpy="0 0 0" />
		<geometry><cylinder radius="0.05" length="1" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"roof.urdf",
			R"(<?xml version="1.0"?><robot name="roof"><link name="roof">
		<collision><origin xyz="0 0 1.02" rpy="0 0 0" />
		<geometry><box size="1 1 0.04" /></geometry>
		</collision></link></robot>)");

		planner.spawn("floor", "floor.urdf", "");

		planner.spawn("robot", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {}, "static",
			{}, "floor", "floor");

		// the robot has two base links :/
		planner.create_collision_ignore_group("floor_base",
			{"robot.base_link", "floor.floor"});

		planner.spawn("sword", "sword.urdf", "", Pose {}, "static",
			{}, "robot", "ee_link");

		planner.create_collision_ignore_group("hand_sword",
			{"robot.wrist_2_link", "robot.wrist_3_link", "sword"});

#ifdef DIFFICULT_PATH_PLANNING_TEST
		double r = 0.4;
#else
		double r = 0.6;
#endif

		planner.spawn("pillar1", "pillar.urdf", "", Pose {r, r});
		planner.spawn("pillar2", "pillar.urdf", "", Pose {-r, r});
		planner.spawn("pillar3", "pillar.urdf", "", Pose {r, -r});
		planner.spawn("pillar4", "pillar.urdf", "", Pose {-r, -r});

#ifdef DIFFICULT_PATH_PLANNING_TEST
		planner.spawn("roof", "roof.urdf", "");
#endif
		planner.set_safety_margin("*", 0.012);

		valarray<double> start = {
			-0.1,
			-1.2,
			2,
			-0.5,
			-1.5 * M_PI,
			-6.2
		};
		valarray<double> target = {
			-0.1 + M_PI,
			1.2 - M_PI,
			-2,
			0.5 - M_PI,
			1.5 * M_PI,
			6.2
		};

		valarray<double> ones = {1, 1, 1, 1, 1, 1};

		auto direct = planner.interpolate(
			{start, target}, 0.04,
			ones * 2, ones * 10, ones * 10, 1.01
		);

		makeDir("exports");
		auto output = planner.render_animation(
			"direct path test", "robot", direct, 0.04,
			"html", "exports/direct.html");

		// planner.export_state("exports/export_test.json");
		// planner.export_state("exports/export_test.msgpack", "msgpack");
		// GestaltPlanner planner2(THIS_TEST_LOG_NAME);
		// planner2.import_state("exports/export_test.json");
		// GestaltPlanner planner3(THIS_TEST_LOG_NAME);
		// planner3.import_state("exports/export_test.msgpack", "msgpack_file");

		// auto output2 = planner2.render_animation(
		// 	"direct path test", "robot", direct, 0.04,
		// 	"html", "exports/direct_export_import.html");

		// auto json11 = planner.render_scene("json test", "robot", "json", "scene1.json");
		// auto json21 = planner2.render_scene("json test", "robot", "json", "scene2.json");
		// auto json31 = planner3.render_scene("json test", "robot", "json", "scene3.json");

		// auto json12 = planner.render_animation(
		// 	"direct path test", "robot", direct, 0.4, "json");
		// auto json22 = planner2.render_animation(
		// 	"direct path test", "robot", direct, 0.4, "json");
		// auto json32 = planner3.render_animation(
		// 	"direct path test", "robot", direct, 0.4, "json");

		// cout << json::diff(json::parse(json11), json::parse(json21)) << "\n";
		// cout << json::diff(json::parse(json12), json::parse(json22)) << "\n";
		// cout << json::diff(json::parse(json11), json::parse(json31)) << "\n";
		// cout << json::diff(json::parse(json12), json::parse(json32)) << "\n";

		// EXPECT_TRUE(output.size() == output2.size());

		EXPECT_EQ(std::hash<std::string> {}(output),
#ifdef DIFFICULT_PATH_PLANNING_TEST
			3807570117450040163ul
#else
			10936402610662019450ul
#endif
			) << " if this test fails because the export has changed, visually inspect the new export and hard code its new hash here.";

		//for (const auto& algorithm : { "BFMT", "BiEST", "BKPIECE1", "EST", "FMT", "KPIECE1", "LazyLBTRRT", "LazyPRM", "LazyPRMstar", "LazyRRT", "LBKPIECE1", "LBTRRT", "PDST", "PRM", "PRMstar", "ProjEST", "RRT", "RRTConnect", "RRTsharp", "RRTstar", "RRTXstatic", "SBL", "SORRTstar", "SPARS", "SPARStwo", "SST", "STRIDE", "TRRT" }) {

		bool foundSolution = false;
		for (const auto& algorithm: {
				 //"KPIECE1",
				 "RRTConnect"
			 }) {

			auto path = planner.plan_path(
				"robot", {target}, start, {}, {},
				1e-6, {}, 1e-6, algorithm, {},
#ifdef DIFFICULT_PATH_PLANNING_TEST
				1e6, 600);
#else
				5000, 5);
#endif

			if (path.size() == 0) {
				cout << algorithm << " found nothing\n";
				continue;
			}

			auto interp = planner.interpolate(
				path, 0.04,
				ones * 2, ones * 10, ones * 10
			);

			planner.render_animation(
				"smart path test", "robot", interp, 0.04, "html",
				string() + "exports/" + algorithm + ".html");

			foundSolution = true;
		}

		EXPECT_TRUE(foundSolution);

		// test planning params
		auto path = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {{"range", "0.5"}},
#ifdef DIFFICULT_PATH_PLANNING_TEST
			1e6, 600);
#else
			5000, 5);
#endif

		// test deterministic planning and waypoints
		auto path1 = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {}, 1000000, inf, 1337, true);

		EXPECT_TRUE(path1.size() > 0);

		auto path2a = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {}, 1000000, inf, 1337, true);

		EXPECT_TRUE(path2a.size() > 0);
		EXPECT_EQ(abs(path1 - path2a).sum().sum(), 0);

		auto path2b = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {}, 1000000, inf, 1338, true); // different random seed

		EXPECT_TRUE(path2b.size() > 0);
		EXPECT_NE(path1[1][0], path2b[2][0]); // make sure paths with different seeds are different

		valarray<valarray<double>> waypoints(&path1[1], 2);

		auto path2c = planner.plan_path(
			"robot", {path1[3]}, start, {}, waypoints, // waypoint suggestions
			1e-6, {}, 1e-6, "RRTConnect", {}, 1000000, inf, 1338, false, false); // different random seed

		EXPECT_TRUE(path2c.size() == 4);
		EXPECT_EQ(abs(path1[1] - path2c[1]).sum(), 0);
		EXPECT_EQ(abs(path1[2] - path2c[2]).sum(), 0);

		// smoothen path
		cout << "tightening...\n";
		auto tightPath = planner.tighten_path("robot", path1);

		cout << "smoothing...\n";
		auto smoothTightPath = planner.smoothen_path("robot", tightPath,
			0.016, ones * 2, ones * 10, ones * 10
		);

		cout << "exporting...\n";

		str::save("exports/found_path.py",
			plot(planner.interpolate(path1, 0.016, ones * 2, {}, {})));

		planner.render_animation(
			"found path", "robot",
			planner.interpolate(path1, 0.016, ones * 2, ones * 10, ones * 10),
			0.016, "html", string() + "exports/found_path.html");

		str::save("exports/tight_path.py",
			plot(planner.interpolate(tightPath, 0.016, ones * 2, {}, {})));

		planner.render_animation(
			"tight path", "robot",
			planner.interpolate(tightPath, 0.016, ones * 2, ones * 10, ones * 10),
			0.016, "html", string() + "exports/tight_path.html");

		// plot(smoothPath, "exports/smooth_path.py");
		// planner.render_animation(
		// 	"smooth path", "robot",
		// 	smoothTightPath, 0.016, "html", string() + "exports/smooth_path.html");

		str::save("exports/smooth_tight_path.py",
			plot(smoothTightPath));

		planner.render_animation(
			"smooth tight path", "robot",
			smoothTightPath, 0.016, "html", string() + "exports/smooth_tight_path.html");

#ifdef DIFFICULT_PATH_PLANNING_TEST
		planner.remove("roof");
#endif

		// test constraints

		start = {
			-0.1,
			-1.2,
			2,
			-0.5,
			-1.5 * M_PI,
			6.2
		};

		target = {
			-0.1 + 2 * M_PI,
			-1.2,
			2,
			-0.5,
			-1.5 * M_PI,
			6.2 - 2 * M_PI
		};

		const valarray<valarray<double>> constraints = {{1, 1, 1, 1, 1, 1}};

		auto path3 = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, constraints, 1e-6, "RRTConnect", {}, 1000000, inf);

		for (const auto& waypoint: path3) {
			EXPECT_FLOAT_EQ(start.sum(), waypoint.sum());
		}

		cout << "tightening...\n";
		auto tightPath3 = planner.tighten_path("robot", path3, constraints);
		cout << "smoothing...\n";
		auto constrainedSmoothTightPath3 = planner.smoothen_path("robot", tightPath3,
			0.016, ones * 2, ones * 10, ones * 10, constraints
		);
		str::save("exports/constrained_smooth_tight_path.py",
			plot(constrainedSmoothTightPath3));
		planner.render_animation(
			"constrained smooth tight path", "robot",
			constrainedSmoothTightPath3, 0.016, "html",
			string() + "exports/constrained_smooth_tight_path.html");
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, plan_many_objects) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		std::vector<btVector3> cubes;
		for (int i = 0; i < 1000; i++) {
			double x = (double) rand() / RAND_MAX * 4 - 2;
			double y = (double) rand() / RAND_MAX * 4 - 2;
			double z = (double) rand() / RAND_MAX * 4 - 2;
			if (x * x + y * y > 0.8 * 0.8 || z < -0.1) {
				cubes.push_back(btVector3(x, y, z));
			}
		}
		std::stringstream cubesUrdf;
		cubesUrdf << "<?xml version=\"1.0\"?>\n";
		cubesUrdf << "<robot name=\"cubes\">\n";
		cubesUrdf << "<link name=\"cubesLink\">\n";
		for (const auto& cube: cubes) {
			cubesUrdf << "<collision>\n";
			cubesUrdf << "<origin xyz=\"" << cube.x() << " " << cube.y() << " " << cube.z() << "\" "
					  << "rpy=\"" << cube.x() << " " << cube.y() << " " << cube.z() << "\" />\n";
			cubesUrdf << "<geometry>\n";
			cubesUrdf << "<box size=\"0.01 0.01 0.01\" />\n";
			cubesUrdf << "</geometry>\n";
			cubesUrdf << "</collision>\n";
		}
		cubesUrdf << "</link>\n";
		cubesUrdf << "</robot>\n";

		planner.cache_file("cubes.urdf", cubesUrdf.str());

		planner.cache_file(
			"sword.urdf",
			R"(<?xml version="1.0"?><robot name="sword"><link name="sword">
		<collision><origin xyz="0.02 0 -0.28" rpy="0 0 0" />
		<geometry><cylinder radius="0.02" length="0.6" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"roof.urdf",
			R"(<?xml version="1.0"?><robot name="roof"><link name="roof">
		<collision><origin xyz="0 0 1.02" rpy="0 0 0" />
		<geometry><box size="1 1 0.04" /></geometry>
		</collision></link></robot>)");

		planner.spawn("cubes", "cubes.urdf", "");

		planner.spawn("robot", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml");

		planner.spawn("sword", "sword.urdf", "", Pose {}, "static",
			{}, "robot", "ee_link");

		planner.create_collision_ignore_group("hand_sword",
			{"robot.wrist_2_link", "robot.wrist_3_link", "sword"});

		planner.set_safety_margin("*", 0.012);

		valarray<double> start = {
			-0.1,
			-1.2,
			2.4,
			-1.2,
			-1.5 * M_PI,
			-6.2
		};
		valarray<double> target = {
			-0.1 + M_PI,
			1.2 - M_PI,
			-2.4,
			1.2 - M_PI,
			1.5 * M_PI,
			6.2
		};

		valarray<double> ones = {1, 1, 1, 1, 1, 1};

		auto direct = planner.interpolate(
			{start, target}, 0.04,
			ones * 2, ones * 10, ones * 10, 1.01
		);

		makeDir("exports");
		auto output = planner.render_animation(
			"direct path test", "robot", direct, 0.04,
			"html", "exports/cubes_direct.html");

		auto start_time = std::chrono::high_resolution_clock::now();

		auto path = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {},
			50000, 60, 0, false, false);

		auto simplePath = planner.simplify_path("robot", path);

		auto tightPath = planner.tighten_path("robot", simplePath);

		auto smoothPath = planner.smoothen_path("robot", tightPath,
			0.016, ones * 2, ones * 10, ones * 10
		);

		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
		std::cout << "Time taken: " << duration << " ms" << std::endl;

		EXPECT_TRUE(smoothPath.size() > 0);

		planner.render_animation(
			"found path", "robot",
			smoothPath,
			0.016, "html", string() + "exports/cubes_found_path.html");

	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}


TEST(test_planner, plan_multistep_path) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.cache_file(
			"floor.urdf",
			R"(<?xml version="1.0"?><robot name="floor"><link name="floor">
		<collision><origin xyz="0 0 -0.02" rpy="0 0 0" />
		<geometry><box size="1 1 0.04" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"sword.urdf",
			R"(<?xml version="1.0"?><robot name="sword"><link name="sword">
		<collision><origin xyz="0.02 0 -0.28" rpy="0 0 0" />
		<geometry><cylinder radius="0.02" length="0.6" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"pillar.urdf",
			R"(<?xml version="1.0"?><robot name="pillar"><link name="pillar">
		<collision><origin xyz="0 0 0.5" rpy="0 0 0" />
		<geometry><cylinder radius="0.05" length="1" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"roof.urdf",
			R"(<?xml version="1.0"?><robot name="roof"><link name="roof">
		<collision><origin xyz="0 0 1.02" rpy="0 0 0" />
		<geometry><box size="1 1 0.04" /></geometry>
		</collision></link></robot>)");

		planner.spawn("floor", "floor.urdf", "");

		planner.spawn("robot", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {}, "static",
			{}, "floor", "floor");

		// the robot has two base links :/
		planner.create_collision_ignore_group("floor_base",
			{"robot.base_link", "floor.floor"});

		planner.spawn("sword", "sword.urdf", "", Pose {}, "static",
			{}, "robot", "ee_link");

		planner.create_collision_ignore_group("hand_sword",
			{"robot.wrist_2_link", "robot.wrist_3_link", "sword"});

#ifdef DIFFICULT_PATH_PLANNING_TEST
		double r = 0.4;
#else
		double r = 0.6;
#endif

		planner.spawn("pillar1", "pillar.urdf", "", Pose {r, r});
		planner.spawn("pillar2", "pillar.urdf", "", Pose {-r, r});
		planner.spawn("pillar3", "pillar.urdf", "", Pose {r, -r});
		planner.spawn("pillar4", "pillar.urdf", "", Pose {-r, -r});

#ifdef DIFFICULT_PATH_PLANNING_TEST
		planner.spawn("roof", "roof.urdf", "");
#endif
		planner.set_safety_margin("*", 0.012);

		valarray<double> start = {-0.1, -1.2, 2, -0.5, -1.5 * M_PI, -6.2};
		valarray<valarray<double>> targets1 = {
			{-0.1 + 0.5 * M_PI, 1.2 - M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 - 1.5 * M_PI, 1.2 - M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 + 0.5 * M_PI, 1.2 + M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 + 0.5 * M_PI, 1.2 - M_PI, -2, 0.5 + M_PI, 1.5 * M_PI, 6.2}
		};
		valarray<valarray<double>> targets2 = {
			{-0.1 + M_PI, 1.2 - M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 - M_PI, 1.2 - M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 + M_PI, 1.2 + M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 + M_PI, 1.2 - M_PI, -2, 0.5 + M_PI, 1.5 * M_PI, 6.2}
		};
		valarray<valarray<double>> targets3 = {
			{-0.1 + 1.5 * M_PI, 1.2 - M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 - 0.5 * M_PI, 1.2 - M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 + 1.5 * M_PI, 1.2 + M_PI, -2, 0.5 - M_PI, 1.5 * M_PI, 6.2},
			{-0.1 + 1.5 * M_PI, 1.2 - M_PI, -2, 0.5 + M_PI, 1.5 * M_PI, 6.2}
		};

		auto paths = planner.plan_multistep_path(
			"robot", {targets1, targets2, targets3}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {},
#ifdef DIFFICULT_PATH_PLANNING_TEST
			1e6, 600
#else
			5000, 5
#endif
		);

		EXPECT_EQ(paths.size(), 3);
		if (paths.size() != 3) { return; }

		size_t equalities;

		equalities = 0;
		for (const auto& t: targets1) { equalities += abs(t - *(std::end(paths[0]) - 1)).sum() == 0; }
		EXPECT_EQ(equalities, 1);

		equalities = 0;
		for (const auto& t: targets2) { equalities += abs(t - *(std::end(paths[1]) - 1)).sum() == 0; }
		EXPECT_EQ(equalities, 1);

		equalities = 0;
		for (const auto& t: targets3) { equalities += abs(t - *(std::end(paths[2]) - 1)).sum() == 0; }
		EXPECT_EQ(equalities, 1);

		EXPECT_TRUE(abs(start - paths[0][0]).sum() == 0);
		EXPECT_TRUE(abs(*(std::end(paths[0]) - 1) - paths[1][0]).sum() == 0);
		EXPECT_TRUE(abs(*(std::end(paths[1]) - 1) - paths[2][0]).sum() == 0);

		// planner.render_animation(
		// 	"constrained smooth tight path", "robot",
		// 	constrainedSmoothTightPath3, 0.016, "html",
		// 	string() + "exports/constrained_smooth_tight_path.html");
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, encapsulate) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.cache_file(
			"floor.urdf",
			R"(<?xml version="1.0"?><robot name="floor"><link name="floor">
		<collision><origin xyz="0 0 -0.02" rpy="0 0 0" />
		<geometry><box size="1 1 0.04" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"sword.urdf",
			R"(<?xml version="1.0"?><robot name="sword"><link name="sword">
		<collision><origin xyz="0.02 0 -0.28" rpy="0 0 0" />
		<geometry><cylinder radius="0.02" length="0.6" /></geometry>
		</collision></link></robot>)");

		planner.cache_file(
			"pillar.urdf",
			R"(<?xml version="1.0"?><robot name="pillar"><link name="pillar">
		<collision><origin xyz="0 0 0.5" rpy="0 0 0" />
		<geometry><cylinder radius="0.05" length="1" /></geometry>
		</collision></link></robot>)");

		// planner.cache_file(
		// 	"roof.urdf",
		// 	R"(<?xml version="1.0"?><robot name="roof"><link name="roof">
		// <collision><origin xyz="0 0 1.02" rpy="0 0 0" />
		// <geometry><box size="1 1 0.04" /></geometry>
		// </collision></link></robot>)");

		planner.spawn("floor", "floor.urdf", "");

		planner.spawn("robot", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {}, "static",
			{}, "floor", "floor", true);

		// the robot has two base links :/
		planner.create_collision_ignore_group("floor_base",
			{"robot.base_link", "floor.floor"});

		planner.spawn("sword", "sword.urdf", "", Pose {}, "static",
			{}, "robot", "ee_link");

		planner.create_collision_ignore_group("hand_sword",
			{"robot.wrist_2_link", "robot.wrist_3_link", "sword"});

		double r = 0.4;

		planner.spawn("pillar1", "pillar.urdf", "", Pose {r, r});
		// planner.spawn("pillar2", "pillar.urdf", "", Pose {-r, r});
		// planner.spawn("pillar3", "pillar.urdf", "", Pose {r, -r});
		// planner.spawn("pillar4", "pillar.urdf", "", Pose {-r, -r});

		// planner.spawn("roof", "roof.urdf", "");

		planner.set_safety_margin("*", 0.012);

		valarray<double> start = {
			-0.1,
			-1.2,
			2,
			-0.5,
			-1.5 * M_PI,
			-6.2
		};
		valarray<double> target = {
			-0.1 + 2 * M_PI,
			-1.2,
			2,
			-0.5,
			-1.5 * M_PI,
			-6.2
		};

		valarray<double> ones = {1, 1, 1, 1, 1, 1};

		makeDir("exports");

		auto path = planner.plan_path(
			"robot", {target}, start, {}, {},
			1e-6, {}, 1e-6, "RRTConnect", {}, 1000000, inf, 0, true, false);

		// auto smoothPath = planner.smoothen_path("robot", path,
		// 	0.016, ones * 2, ones * 10, ones * 10
		// );

		// str::save("exports/encapsulated_plan.py",
		// 	plot(smoothPath));
		// planner.render_animation(
		// 	"encapsulated path", "robot", smoothPath,
		// 	0.016, "html", string() + "exports/encapsulated_plan.html");
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, interpolate) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);

#ifdef MANY_INTERPOLATION_TESTS
		size_t nTrials = 30000;
#else
		size_t nTrials = 30;
#endif

		for (size_t iTrial = 0; iTrial < nTrials; iTrial++) {
			cout << iTrial << " " << std::flush;

			size_t np = round(random(2, 10));
			size_t dof = round(random(1, 8));
			valarray<valarray<double>> waypoints(np);
			for (size_t i = 0; i < np; i++) {
				waypoints[i] = randoms(dof, -10, 10);
			}
			valarray<double> max_velocity = randoms(dof, 1, 10);
			valarray<double> max_acceleration = randoms(dof, 1, 10);
			valarray<double> max_jerk = randoms(dof, 1, 10);

			double safety = random(1, 1.1);
			valarray<double> safe_max_velocity = max_velocity * (1.00001 / safety);
			valarray<double> safe_max_acceleration = max_acceleration * (1.00001 / safety);
			valarray<double> safe_max_jerk = max_jerk * (1.00001 / safety);

			double dt = random(0.01, 0.001);

			auto interpolated = planner.interpolate(
				waypoints, dt, max_velocity, max_acceleration, max_jerk, safety);

			auto pos = interpolated;
			auto vel = diff(pos) / dt;
			auto acc = diff(vel) / dt;
			auto jrk = diff(acc) / dt;

			// make sure we moved from start to target
			EXPECT_EQ(abs(waypoints[0] - pos[0]).sum(), 0);
			EXPECT_LT(abs(
						  waypoints[waypoints.size() - 1]
						  - pos[pos.size() - 1]
						  ) .sum(), 1e-12);

			// assert that kinematic limits are obeyed

			valarray<double> lastDirection(0.0, dof);
			size_t directionChangesInARow = 0;

			for (const auto& v: vel) {
				double lengthSquared = 0;
				for (auto&& [vis, vmax]: zip(v, safe_max_velocity)) {
					// speed limit
					EXPECT_LE(fabs(vis), vmax);
					lengthSquared += vis * vis;
				}
				// assert straight line in configuration space
				if (lengthSquared > 1e-12) {
					double length = sqrt(lengthSquared);
					valarray<double> direction = v;
					for (auto& d: direction) {
						d /= length;
					}
					bool directionChange = false;
					for (auto&& [d0, d1]: zip(lastDirection, direction)) {
						if (fabs(d1 - d0) > 1e-6) {
							directionChange = true;
						}
					}
					lastDirection = direction;
					if (directionChange) {
						directionChangesInARow++;
					}
					else {
						directionChangesInARow = 0;
					}
					EXPECT_LE(directionChangesInARow, 2);
				}
			}

			// acceleration limit
			for (const auto& a: acc) {
				for (auto&& [ais, amax]: zip(a, safe_max_acceleration)) {
					EXPECT_LE(fabs(ais), amax);
				}
			}

			// jerk limit
			for (const auto& j: jrk) {
				for (auto&& [jis, jmax]: zip(j, safe_max_jerk)) {
					EXPECT_LE(fabs(jis), jmax);
				}
			}

			// make sure that the time we are not at some limit
			// is at maximum a few steps
			size_t lastLimit = 0;
			for (size_t i = 0; i < jrk.size(); i++) {
				for (size_t j = 0; j < dof; j++) {
					if (
						fabs(fabs(vel[i][j]) - safe_max_velocity[j]) < 1e-3
						|| fabs(fabs(acc[i][j]) - safe_max_acceleration[j]) < 1e-3
						|| fabs(fabs(jrk[i][j]) - safe_max_jerk[j]) < 1e-3
					) {
						lastLimit = i;
						break;
					}
				}
				// EXPECT_GE(lastLimit + 5, i);
			}
		}
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, smoothen) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);

		valarray<valarray<double>> path = {
			{1, 0},
			{2, 10},
			{3, 10.1},
			{4, 20},
			{5, 5}
		};

		// valarray<valarray<double>> smoothPath = planner.smoothen_path("", path);

		// plot(smoothPath, "smooth.py");
		// plot(diff(smoothPath), "smoothv.py");
		// plot(diff(diff(smoothPath)), "smootha.py");
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, checks) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.cache_file(
			"floor.urdf",
			R"(<?xml version="1.0"?><robot name="floor"><link name="floor">
		<collision><origin xyz="0.25 0 -0.02" rpy="0 0 0" />
		<geometry><cylinder radius="1" length="0.04" /></geometry>
		</collision></link></robot>)");

		planner.spawn("floor", "floor.urdf");

		planner.spawn("robot1", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {});

		planner.spawn("robot2", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {0.4});

		planner.create_collision_ignore_group("floor_bases",
			{"robot1.base_link", "robot2.base_link", "floor.floor"});

		planner.set_safety_margin("*", 0.012);

		valarray<double> start = {-1, -1, 1, 0, 0, 0};

		planner.set_joint_positions("robot1", start);
		planner.set_joint_positions("robot2", start);

		// the two robots should collide in the middle step
		valarray<valarray<double>> traj {
			start,
			{0, -1, 1, 0, 0, 0},
			{1, -1, 1, 0, 0, 0}
		};

		EXPECT_EQ(planner.find_collisions().size(), 0);
		EXPECT_EQ(planner.find_collisions("robot1", traj).size(), 1);
		EXPECT_FALSE(planner.check_clearance("robot1", traj));

		// bury robot2 in the floor
		planner.set_joint_positions("robot2", {0, 1, 0, 0, 0, 0});

		EXPECT_EQ(planner.find_collisions().size(), 1);

		// robot1 should not collide, robot2 is passive
		EXPECT_EQ(planner.find_collisions("robot1", traj).size(), 0);
		EXPECT_TRUE(planner.check_clearance("robot1", traj));

		valarray<double> maxes = {1, 2, 3, 4, 5, 6};

		double dt = 0.01;
		auto inter = planner.interpolate(traj, dt, maxes, maxes, maxes, 1.0001);

		EXPECT_TRUE(planner.check_kinematic_feasibility(
			inter, dt, maxes, maxes, maxes));
		EXPECT_FALSE(planner.check_kinematic_feasibility(
			inter, dt * 0.999, maxes, maxes, maxes));
	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}

TEST(test_planner, concave) {
	{
		GestaltPlanner planner(THIS_TEST_LOG_NAME);
		auto& state = *GestaltPlannerTest::getState(planner);

		planner.spawn("robot1", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {-0.65, -0.35},
			"static", {0, -1, 1, 0, 0, 0});

		planner.spawn("c1", "../../../src/test/urdf/concave_c.urdf", "",
			Pose(), "static", {}, "robot1", "ee_link"
		);

		planner.spawn("robot2", "../../../models/ur5e/ur5e.urdf",
			"../../../models/ur5e/ur5e.yaml", Pose {0.65, 0.35, 0.05, 0, 0, 1, 0},
			"static", {0, -1, 1, 0, 0, 1.57});

		planner.spawn("c2", "../../../src/test/urdf/concave_c_with_holes.urdf", "",
			Pose(), "static", {}, "robot2", "ee_link"
		);

		planner.set_safety_margin("*", 0.01);

		EXPECT_TRUE(planner.check_clearance());

		makeDir("exports");
		planner.render_scene("concave mesh test", "__world__", "html", "exports/concave_clear.html");

		planner.set_joint_positions("robot1", {0, -1, 1, 0, 0, 0.8});
		EXPECT_FALSE(planner.check_clearance("robot1"));

		planner.render_scene("concave mesh test", "__world__", "html", "exports/concave_colliding.html");
	
		valarray<double> ones = {1, 1, 1, 1, 1, 1};
		auto direct = planner.interpolate(
			{{0, -1, 1, 0, 0, 0}, {0, -1, 1, 0, 0, 3.14}}, 0.04,
			ones * 2, ones * 10, ones * 10, 1.01
		);

		planner.render_animation(
			"concave mesh - direct path test", "robot1", direct, 0.04,
			"html", "exports/concave_direct_motion.html");

		auto path = planner.plan_path(
			"robot1", {{0, -1, 1, 0, 0, 0}}, {0, -1, 1, 0, 0, 3.14},
			{}, {}, 1e-6, {}, 1e-6, "RRTConnect", {}, 1000000, inf, 0, true, false);

		// smoothen path
		auto tightPath = planner.tighten_path("robot1", path, {}, 10);

		auto smoothTightPath = planner.smoothen_path("robot1", tightPath,
			0.016, ones * 2, ones * 10, ones * 10
		);

		planner.render_animation(
			"concave mesh - found path", "robot1", smoothTightPath, 0.04,
			"html", "exports/concave_found_path.html");

	}
#ifdef BT_DEBUG_MEMORY_ALLOCATIONS
	btDumpMemoryLeaks();
#endif
}
#endif
TEST_MAIN
