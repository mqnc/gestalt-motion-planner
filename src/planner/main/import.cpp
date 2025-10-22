
// #include "planner_headers.h"
// // #include "serialization/plannerstate.hpp"

// /*( public: )*/ void GestaltPlanner :: import_state(
// 	const string& input,
// 	const string& input_format /*( = "json_file" )*/ // "json_file" / "json_string" / "msgpack_file" / "msgpack_string"
// ) {
// 	auto guard = state->log.log("gp.import_state",
// 		input, input_format);

// 	if (input_format != "json_file"
// 		&& input_format != "json_string"
// 		&& input_format != "msgpack_file"
// 		&& input_format != "msgpack_string"
// 		) {
// 		throw runtime_error(string() + "unknown format: " + input_format);
// 	}

// 	string raw;
// 	bool isFile = input_format == "json_file" || input_format == "msgpack_file";
// 	bool isJson = input_format == "json_file" || input_format == "json_string";
// 	bool isBinary = !isJson;

// 	if (isFile) {
// 		auto dir = std::filesystem::path(input).parent_path();
// 		raw = state->loadFile(input, isBinary);
// 	}
// 	else {
// 		raw = input;
// 	}

// 	// if (isJson) {
// 	// 	json::parse(raw).get_to(*state);
// 	// }
// 	// else {
// 	// 	json::from_msgpack(raw).get_to(*state);
// 	// }
// }
