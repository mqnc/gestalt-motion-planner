// precompiled headers

#pragma once

#include "stl.h"
#include "str.h"
#include "utils.h"
#include "valarraytools.h"
#include "zip.h"
#include "registry.h"

#define JSON_HANDLE_INF_NAN
#include <json_mod.hpp>

using nlohmann::json;

template<typename T>
inline void to_json(json& j, const unique_ptr<T>& p) {
	j = *p;
}
