
#pragma once

#ifdef PROFILING

#include <string>
#include <vector>
#include <unordered_map>
#include <iostream>
#include <sstream>
#include <tuple>
#include <algorithm>
#include <functional>
#include "timer.h"

class Profiling {
	inline static Timer programTimer {false};
	inline static std::unordered_map<std::string, Timer> timers;

public:

	// so create() and run() return something so they can be members so we can use them for ctor dtor timing
	struct Stub {};

	static Stub create(std::string name) {
		timers.try_emplace(name, false);
		return Stub{};
	}

	static Stub run(std::string name) {
		create(name);
		timers.at(name).run();
		return Stub{};
	}

	static void pause(std::string name) {
		timers.at(name).pause();
	}

	struct PauseOnScopeExit {
		std::string name;
		~PauseOnScopeExit() { Profiling::pause(name); }
	};

	static std::string report() {
		std::vector<std::pair<std::string, Timer>> timerVector(timers.begin(), timers.end());

		std::sort(timerVector.begin(), timerVector.end(),
			[](const auto& a, const auto& b) {
				return a.second.s() > b.second.s();
			});

		std::stringstream ss;

		ss << "\n" << programTimer << " (100.00 %) program runtime\n";
		ss << "-------------------------------------------\n";

		for (const auto& [name, timer]: timerVector) {
			ss << timer << " (" << (100 * timer.s() / programTimer.s()) << " %) " << name << "\n";
		}

		return ss.str();
	}

	static void reset() {
		programTimer.reset();
		timers.clear();
		programTimer.run();
	}

	Profiling() {
		programTimer.run();
	}

	~Profiling() {
		std::cout << report();
	}
};

inline Profiling profilerSingleton;

#define PROFILE_RUN(NAME) Profiling::run(#NAME)
#define PROFILE_PAUSE(NAME) Profiling::pause(#NAME)
#define PROFILE_SCOPE(NAME) Profiling::run(#NAME); Profiling::PauseOnScopeExit profiling___stopper = Profiling::PauseOnScopeExit{#NAME}

#define PROFILE_CTOR_FIRST_MEMBER(NAME) Profiling::Stub profiling___stub = Profiling::run(#NAME)
#define PROFILE_CTOR_STOP(NAME) Profiling::pause(#NAME)
#define PROFILE_DTOR_START(NAME) Profiling::run(#NAME)
#define PROFILE_DTOR_LAST_MEMBER(NAME) Profiling::PauseOnScopeExit profiling___stopper = Profiling::PauseOnScopeExit{#NAME}

#define PROFILE_REPORT Profiling::report()
#define PROFILE_RESET Profiling::reset()

#else

#define PROFILE_RUN(NAME)
#define PROFILE_PAUSE(NAME)
#define PROFILE_SCOPE(NAME)

#define PROFILE_CTOR_FIRST_MEMBER(NAME)
#define PROFILE_CTOR_STOP(NAME)
#define PROFILE_DTOR_START(NAME)
#define PROFILE_DTOR_LAST_MEMBER(NAME)

#define PROFILE_REPORT
#define PROFILE_RESET

#endif