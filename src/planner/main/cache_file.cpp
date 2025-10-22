
#include "planner_headers.h"

/*( public: )*/ void GestaltPlanner::cache_file(
	const string& file_name,
	const string& raw_content,
	bool is_base64 /*( = false )*/
) {
	auto guard = state->log.log("gp.cache_file",
		file_name, raw_content, is_base64);

	if (state->isCached(file_name)) {
		cout << "warning: " << file_name << " is already cached, overwriting\n";
	}
	state->cacheFile(file_name, raw_content, is_base64);
}
