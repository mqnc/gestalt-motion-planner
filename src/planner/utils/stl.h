
#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <numeric>
#include <optional>
#include <tuple>
#include <random>
#include <sstream>
#include <stdexcept>
#include <stdlib.h>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <valarray>
#include <vector>

// this is arguably bad practice but it unclutters the code considerably

using std::cout;
using std::string;
using std::literals::string_literals::operator""s;
using std::stringstream;
using std::to_string;

using std::array;
using std::vector;
using std::valarray;
using std::function;
using std::optional;
using std::pair;

template<typename T>
using dict = std::unordered_map<std::string, T>;

using std::unordered_set;

using std::unique_ptr;
using std::make_unique;
using std::move;
using std::shared_ptr;
using std::make_shared;
using std::weak_ptr;

using std::runtime_error;

const auto NaN = std::nan("");
const auto inf = std::numeric_limits<double>::infinity();
using std::abs;
using std::min;
using std::max;
using std::begin;
using std::end;
using std::isinf;
using std::isnan;
