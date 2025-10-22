
#include "planner_headers.h"
#include "collision/shapes.h"
#include "render/meshgenerator.h"
#include "render/sceneexport.hpp"
#include "render/toJson.hpp"
#include "render/wraphtml.h"
#include "profiling.h"

/*( private: )*/ string GestaltPlanner::render_impl(
	bool isAnimation,
	const string& title,
	const string& object_id,
	const valarray<valarray<double>>& trajectory,
	double dt,
	const string& format,
	const string& output_file,
	bool probe_margins
) {
	PROFILE_SCOPE(rendering);
	Scene scene;
	scene.title = title;
	scene.numSteps = trajectory.size();
	scene.dt = dt;
	auto [meshes, links] = getParts(*state);
	scene.objects = links;

	if (object_id != "") {
		// some object is marked actuated but we export only a static scene
		if (trajectory.size() == 0) {
			auto& robot = state->getRobot(object_id);

			robot.setActuationState(true);
			OnScopeExit deactuator(
				[&]() {robot.setActuationState(false);});
			update_collision_bitmasks();
			auto active = listActuatedParts(*state);
			scene.collisions = { getCollisionInfo(*state) };
			robot.setActuationState(false);

			for (const auto& [name, ref] : active) {
				for (const auto& meshType : { ".margin", ".hull" }) {
					const auto& typedName = name + meshType;
					scene.objects.at(typedName).isActuated = true;
				}
			}
		}
		// export animation
		else {
			auto& robot = state->getRobot(object_id);

			robot.setActuationState(true);
			OnScopeExit deactuator(
				[&]() {robot.setActuationState(false);});
			update_collision_bitmasks();

			animateLinks(*state, object_id, trajectory,
				scene.objects, scene.collisions);
		}
	}


	string result;

	if ((format == "json") || (format == "js") || (format == "html")) {
		string json = toJson(meshes, { scene });

		if (format == "json") {
			result = json;
		}
		else if (format == "js") {
			result = "let sceneData=" + json + ";";
		}
		else if (format == "html") {
			result = wrapHtml(json);
		}
	}
	else {
		throw runtime_error("only json, js and html are supported export formats at the moment");
	}

	if (output_file != "") {
		str::save(output_file, result);
	}
	return result;

}

/*( public: )*/ string GestaltPlanner::render_scene(
	const string& title /*( = "debug" )*/,
	const string& active_object /*( = "__world__" )*/,
	const string& format /*( = "json" )*/,
	const string& output_file /*( = "" )*/, // leave empty to only return string
	bool probe_margins /*( = false )*/
) {
	auto guard = state->log.log("gp.render_scene",
		title, active_object, format, output_file, probe_margins);

	return render_impl(
		false,
		title,
		active_object,
		{},
		0,
		format,
		output_file,
		probe_margins
	);
}

/*( public: )*/ string GestaltPlanner::render_animation(
	const string& title /*( = "debug" )*/,
	const string& active_object /*( = "__world__" )*/,
	const valarray<valarray<double>>& trajectory /*( = {} )*/,
	double dt /*( = 1.0 )*/,
	const string& format /*( = "json" )*/,
	const string& output_file /*( = "" )*/ // leave empty to only return string
) {
	auto guard = state->log.log("gp.render_animation",
		title, active_object, trajectory, dt, format, output_file);

	return render_impl(
		true,
		title,
		active_object,
		trajectory,
		dt,
		format,
		output_file,
		false
	);
}
