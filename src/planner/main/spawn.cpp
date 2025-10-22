
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::spawn(
	const string& object_id,
	const string& description_file,
	const string& config_file /*( = "" )*/,
	const Pose& pose /*( = Pose() )*/,
	const string& pose_type /*( = "static" )*/,
	const valarray<double>& joint_positions /*( = {} )*/,
	const string& parent_object_id /*( = "__world__" )*/,
	const string& parent_link /*( = "__root__" )*/,
	bool encapsulate_meshes /*( = false )*/
) {
	state->log.log(Log::Raw {"// deferring the following spawn call to cache files first\n// "});
	state->log.log("gp.spawn",
		object_id, description_file, config_file,
		pose, pose_type, joint_positions,
		parent_object_id, parent_link);
	state->log.log(Log::Raw {"// (gp.spawn not actually done)\n"});

	if (object_id.find("__", 0) == 0) {
		throw runtime_error(object_id
			+ " ids starting with '__' are reserved");
	}

	state->assertIdFree(object_id);

	if (pose_type == "virtual") {
		return;
	}
	auto dir = std::filesystem::path(description_file).parent_path();
	auto urdf_source = state->loadFile(description_file);
	auto urdf = parseUrdf(urdf_source);

	vector<string> actuatedJoints = urdf.defaultJointSelection;

	dict<string> meshSources = {};
	for (const auto& link: urdf.links) {
		for (const auto& geom: link.collisionGeometries) {
			const auto& f = geom.filename;
			if (f != "") {
				if (std::filesystem::path(f).is_absolute()) {
					meshSources[f] = state->loadFile(f, true);
				}
				else {
					meshSources[f] = state->loadFile(dir / f, true);
				}
			}
		}
	}

	dict<vector<string>> collisionIgnoreGroups = {};

	if (config_file != "") {
		actuatedJoints = {};
		try {
			auto config_source = state->loadFile(config_file);
			YAML::Node config = YAML::Load(config_source);
			if (config["actuated_joints"]) {
				for (const auto& j: config["actuated_joints"]) {
					actuatedJoints.push_back(j.as<string>());
				}
			}
			if (config["ignore_collisions"]) {
				for (const auto& group_kv: config["ignore_collisions"]) {
					vector<string> myGroup;
					for (const auto& item: group_kv.second) {
						myGroup.push_back(item.as<string>());
					}
					collisionIgnoreGroups[group_kv.first.as<string>()] = myGroup;
				}
			}
		}
		catch (runtime_error& e) {
			cout << "error loading configuration file "
				 << config_file << ":\n" << e.what() << "\n";
		}
	}

	state->log.log(Log::Raw {"// deferred spawn call:\n"});
	auto guard2 = state->log.log("gp.spawn",
		object_id, description_file, config_file,
		pose, pose_type, joint_positions,
		parent_object_id, parent_link);

	state->robotTemplates[object_id] =
		make_shared<CollisionRobotTemplate>(object_id,
			urdf, meshSources, actuatedJoints, collisionIgnoreGroups,
			encapsulate_meshes);

	state->robots.emplace(object_id,
		CollisionRobot(*state->robotTemplates.at(object_id), object_id));

	auto& robot = state->getRobot(object_id);

	auto& parent = state->getRobot(parent_object_id);
	robot.setParent(&parent, parent_link);
	robot.setBaseTrafo(btTransformFromPose(pose));

	if (joint_positions.size() > 0) {
		robot.setJointPositions(joint_positions);
	}

	for (const auto& group_kv: robot.getCollisionIgnoreGroups()) {
		state->collisionIgnoreGroupManager.createGroup(
			group_kv.first, group_kv.second);
	}
}
