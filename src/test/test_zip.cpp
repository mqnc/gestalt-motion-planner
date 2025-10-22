template<typename T>
struct which_type;

#define WHICH_TYPE(x) which_type<x>()
#define WHICH_VALUE_TYPE(x) which_type<decltype(x)>()


#include "test_main.h"
#include "stl.h"
#define ALLOW_PARALLEL_ITERATION_THROUGH_INDEPENDENT_MAPS
#include "zip.h"
#include <type_traits>

#define ASSERT_CONST(x) static_assert( \
	std::is_const_v<std::remove_reference_t<decltype(x)>>, \
	"type is not constant")
#define ASSERT_MUTABLE(x) static_assert( \
	!std::is_const_v<std::remove_reference_t<decltype(x)>>, \
	"type is not mutable")

TEST(test_zip, complete) {

	vector<double> vec = { 1 };
	const vector<double> cvec = { 4, 5, 6 };
	dict<double> map = { {"x", 7}, {"y", 7.5}, {"z", 8} };
	dict<const double> mapc = { {"x", 9}, {"y", 10} };
	const dict<double> cmap = { {"x", 11}, {"y", 12} };

	// standard map for comparison
	for (auto [k, v] : std::map<int, int>{ {0, 0} }) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
	}

	for (auto& [k, v] : std::map<int, int>{ {0, 0} }) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
	}

	for (auto&& [k, v] : std::map<int, int>{ {0, 0} }) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
	}

	for (const auto [k, v] : std::map<int, int>{ {0, 0} }) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
	}

	for (const auto& [k, v] : std::map<int, int>{ {0, 0} }) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
	}

	// for (const auto&& [k, v] : std::map<int, int>{ {0, 0} }) {
	// 	ASSERT_CONST(k);
	// 	ASSERT_CONST(v);
	// }

	for (auto [k, v] : map) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
		ASSERT_NE(&(map[k]), &v);
	}

	for (auto& [k, v] : map) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
		ASSERT_EQ(&(map[k]), &v);
	}

	for (auto&& [k, v] : map) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
		ASSERT_EQ(&(map[k]), &v);
	}

	for (const auto [k, v] : map) {
		ASSERT_CONST(v);
		ASSERT_NE(&(map[k]), &v);
	}

	for (const auto& [k, v] : map) {
		ASSERT_CONST(v);
		ASSERT_EQ(&(map[k]), &v);
	}

	// for (const auto&& [k, v] : map) {
	// 	ASSERT_CONST(v);
	// 	ASSERT_EQ(&(map[k]), &v);
	// }

	for (auto [k, v] : cmap) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
		ASSERT_NE(&(cmap.at(k)), &v);
	}

	for (auto& [k, v] : cmap) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
		ASSERT_EQ(&(cmap.at(k)), &v);
	}

	for (auto&& [k, v] : cmap) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
		ASSERT_EQ(&(cmap.at(k)), &v);
	}

	for (auto [k, v] : mapc) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
		ASSERT_NE(&(mapc.at(k)), &v);
	}

	for (auto& [k, v] : mapc) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
		ASSERT_EQ(&(mapc.at(k)), &v);
	}

	for (auto&& [k, v] : mapc) {
		ASSERT_CONST(k);
		ASSERT_CONST(v);
		ASSERT_EQ(&(mapc.at(k)), &v);
	}

	// zip iterators
	{
		size_t i = 0;
		for (auto&& [v, c] : zip(vec, cvec)) {
			ASSERT_MUTABLE(v);
			ASSERT_CONST(c);
			ASSERT_EQ(&(vec[i]), &v);
			ASSERT_EQ(&(cvec[i]), &c);
			i++;
		}
	}

	for (auto&& [v, c] : zip(
		vector<int>{1, 2, 3},
		array<const int, 3>{1, 2, 3}
	)) {
		ASSERT_MUTABLE(v);
		ASSERT_CONST(c);
	}

	{
		size_t i = 0;
		for (auto&& [v, c] : zip(std::as_const(vec), cvec)) {
			ASSERT_CONST(v);
			ASSERT_CONST(c);
			ASSERT_EQ(&(vec[i]), &v);
			ASSERT_EQ(&(cvec[i]), &c);
			i++;
		}
	}
	// todo: for (const auto&& ...) doesn't make anything const
	// although it works for regular maps without zip

	for (auto&& [x, k, v] : zip(vec, map)) {
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
		ASSERT_EQ(&(map[k]), &v);
	}

	#ifdef ALLOW_PARALLEL_ITERATION_THROUGH_INDEPENDENT_MAPS
	// I can't think of a scenario where this is useful,
	// k1 and k2 can be unordered in unordered_maps
	for (auto&& [k1, v1, k2, v2] : zip(cmap, mapc)) {
		ASSERT_CONST(k1);
		ASSERT_CONST(v1);
		ASSERT_CONST(k2);
		ASSERT_CONST(v2);
		ASSERT_EQ(&(cmap.at(k1)), &v1);
		ASSERT_EQ(&(mapc.at(k2)), &v2);
	}
	#endif

	for (auto&& [i, v] : enumerate(vec)) {
		ASSERT_CONST(i);
		ASSERT_MUTABLE(v);
		ASSERT_EQ(&(vec[i]), &v);
	}

	for (auto&& [i, c] : enumerate(cvec)) {
		ASSERT_CONST(i);
		ASSERT_CONST(c);
		ASSERT_EQ(&(cvec[i]), &c);
	}

	for (auto&& [i, k, v] : enumerate(map)) {
		ASSERT_CONST(i);
		ASSERT_CONST(k);
		ASSERT_MUTABLE(v);
		ASSERT_EQ(&(map[k]), &v);
	}

	{
		size_t i = 0;
		for (auto&& [sep, k, v] : insert(",").between(map)) {
			ASSERT_CONST(sep);
			ASSERT_EQ(sep, i == 0 ? std::string("") : std::string(","));
			ASSERT_CONST(k);
			ASSERT_MUTABLE(v);
			ASSERT_EQ(&(map[k]), &v);
			i++;
		}
	}

	{
		size_t i = 0;
		for (auto&& [sep, k, v] : commasep(map)) {
			ASSERT_CONST(sep);
			ASSERT_EQ(sep, i == 0 ? std::string("") : std::string(","));
			ASSERT_CONST(k);
			ASSERT_MUTABLE(v);
			ASSERT_EQ(&(map[k]), &v);
			i++;
		}
	}
}

TEST_MAIN
