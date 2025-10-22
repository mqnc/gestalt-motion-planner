// #include "planner_headers.h"
// // #include "serialization/plannerstate.hpp"

// /*( public: )*/ string GestaltPlanner :: export_state(
// 	const string& output_file /*( = "" )*/, // leave empty to only return string
// 	const string& output_format /*( = "json" )*/ // "json" / "msgpack"
// ) {
// 	auto guard = state->log.log("gp.export_state",
// 		output_file, output_format);

// 	// string result;
// 	// bool binary = false;

// 	// if (output_format == "json") {
// 	// 	result = json(*state).dump();
// 	// 	binary = false;
// 	// }
// 	// else if (output_format == "msgpack") {
// 	// 	auto pack = json::to_msgpack(json(*state));
// 	// 	result = string(pack.begin(), pack.end());
// 	// 	binary = true;
// 	// }
// 	// else {
// 	// 	throw runtime_error(string() + "unknown format: " + output_format);
// 	// }

// 	// if (output_file != "") {
// 	// 	str::save(output_file, result, binary);
// 	// }
// 	// return result;
// }
