
/*
this file provides template magic
for iterating through two containers
simultaneously in for loops
*/

#pragma once

#include <tuple>
#include <string>

template <typename> struct is_pair : std::false_type {};
template <typename K, typename V> struct is_pair<std::pair<K, V>> : std::true_type {};
template <typename T> struct is_pair<T&> : is_pair<T> {};
template <typename T> struct is_pair<const T> : is_pair<T> {};
template <typename T> struct is_pair<volatile T> : is_pair<T> {};

template<typename T1, typename T2>
class Zipper {
	T1 container1;
	T2 container2;

	template<typename I1, typename I2>
	class Iter {
		I1 iterator1;
		I2 iterator2;

		template<typename U, typename V,
			typename std::enable_if<!is_pair<U>::value, int>::type = 0,
			typename std::enable_if<!is_pair<V>::value, int>::type = 0>
			auto wrap(U&& u, V&& v) {
			return std::forward_as_tuple(u, v);
		}

		template<typename U, typename V,
			typename std::enable_if<is_pair<U>::value, int>::type = 0,
			typename std::enable_if<!is_pair<V>::value, int>::type = 0>
			auto wrap(U&& u, V&& v) {
			return std::forward_as_tuple(u.first, u.second, v);
		}

		template<typename U, typename V,
			typename std::enable_if<!is_pair<U>::value, int>::type = 0,
			typename std::enable_if<is_pair<V>::value, int>::type = 0>
			auto wrap(U&& u, V&& v) {
			return std::forward_as_tuple(u, v.first, v.second);
		}

		template<typename U, typename V,
			typename std::enable_if<is_pair<U>::value, int>::type = 0,
			typename std::enable_if<is_pair<V>::value, int>::type = 0>
			auto wrap(U&& u, V&& v) {
#ifndef ALLOW_PARALLEL_ITERATION_THROUGH_INDEPENDENT_MAPS
			static_assert(!is_pair<U>::value, "\n\n\
you probably want\n\n\
for(auto&& [k, v] : map1){\n\
\tf(map1[k], map2[k]);\n\
}\n\n\
instead of\n\n\
for(auto&& [k1, v1, k2, v2] : zip(map1, map2)){\n\
\tf(v1, v2);\n\
}\n\n\
otherwise #define ALLOW_PARALLEL_ITERATION_THROUGH_INDEPENDENT_MAPS\n");
#endif
			return std::forward_as_tuple(u.first, u.second, v.first, v.second);
		}

	public:
		template<typename I1Fwd, typename I2Fwd>
		Iter(I1Fwd&& iterator1, I2Fwd&& iterator2) :
			iterator1(std::forward<I1Fwd>(iterator1)),
			iterator2(std::forward<I2Fwd>(iterator2)) {}

		void operator++() {
			++iterator1;
			++iterator2;
		}
		auto operator*() {
			return wrap(*iterator1, *iterator2);
		}
		bool operator!=(const Iter& other) {
			return iterator1 != other.iterator1
				&& iterator2 != other.iterator2;
		}
	};
public:
	template<typename T1Fwd, typename T2Fwd>
	Zipper(T1Fwd&& c1, T2Fwd&& c2) :
		container1(std::forward<T1Fwd>(c1)),
		container2(std::forward<T2Fwd>(c2)) {}

	template<typename I1, typename I2>
	auto make_iterator(I1&& iterator1, I2&& iterator2) -> Iter<I1, I2> {
		return { std::forward<I1>(iterator1), std::forward<I2>(iterator2) };
	}

	auto begin() {
		//which_type<decltype(container1.begin())>();
		return make_iterator(
			std::begin(container1),
			std::begin(container2)
		);
	}
	auto end() {
		return make_iterator(
			std::end(container1),
			std::end(container2)
		);
	}
};

template<typename T1, typename T2>
auto zip(T1&& container1, T2&& container2) -> Zipper<T1, T2> {
	return { std::forward<T1>(container1), std::forward<T2>(container2) };
}



class Counter {
	class Iter {
	public:
		size_t i;
		void operator++() { ++i; }
		const size_t& operator*() { return i; }
		bool operator!=(const Iter& other) {
			return i != other.i;
		}
	};

	size_t limit;
public:
	Counter(size_t limit = size_t(-1)) :limit(limit) {}
	Iter begin() { return Iter(); }
	Iter end() { return Iter{ limit }; }
};

template<typename T>
auto enumerate(T&& container) -> Zipper<Counter, T> {
	return { Counter(), std::forward<T>(container) };
}



template<typename T>
class FirstThen {
	class Iter {
	public:
		T first;
		T then;
		bool isFirst = true;
		void operator++() { isFirst = false; }
		const auto& operator*() { return isFirst ? first : then; }
		bool operator!=(const Iter&) {
			return true;
		}
	};

	T first;
	T then;
public:
	FirstThen(T first, T then) :first{ first }, then{ then } {}
	Iter begin() { return Iter{ first, then }; }
	Iter end() { return Iter(); }
};

class insert {
	std::string sep;
public:
	insert(std::string separator) :sep(separator) {}
	template<typename T>
	auto between(T&& container) {
		return zip(FirstThen(std::string(""), sep), std::forward<T>(container));
	}
};

template<typename T>
auto commasep(T&& container) {
	return insert(",").between(std::forward<T>(container));
}