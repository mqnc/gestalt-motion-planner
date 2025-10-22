
#pragma once

#include "common.h"

class CollisionIgnoreGroupManager {

	dict<unordered_set<string>> registry;
	dict<BitMask> bitMasks;
	bool ready = false;

public:

	template<template<class> class Cont>
	void createGroup(
		const string& name,
		const Cont<string>& memberIds
	) {
		ready = false;

		if (registry.size() >= MAX_BIT) {
			throw runtime_error(
				string("maximum number of collision ignore groups (")
				+ to_string(MAX_BIT + 1) + ") exceeded"
			);
		}

		if (registry.count(name) > 0) {
			throw runtime_error(
				string("collision ignore group \"")
				+ name + "\" already exists"
			);
		}

		registry[name]={};
		for(const auto &id:memberIds){
			registry[name].insert(id);
		}
	}

	template<template<class> class Cont>
	void resetGroup(const string& name,
		const Cont<string>& memberIds
	) {
		ready = false;

		if (registry.count(name) == 0) {
			throw runtime_error(
				string("collision ignore group \"")
				+ name + "\" does not exist"
			);
		}

		registry[name]={};
		for(const auto &id:memberIds){
			registry[name].insert(id);
		}
	}

	void deleteGroup(const string& name) {
		ready = false;

		if (registry.count(name) == 0) {
			cout << "available collision ignore groups:\n";
			for(const auto& [id, _]:registry){
				cout << id << " ";
			}
			throw runtime_error(
				string("collision ignore group \"")
				+ name + "\" does not exist"
			);
		}
		registry.erase(name);
	}

	void generateBitMasks() {
		bitMasks.clear();
		assert(registry.size() <= MAX_BIT);

		BitMask bit = 1;
		for (const auto& group_kv : registry) {
			for (const auto& id : group_kv.second) {
				if (bitMasks.count(id) == 0) {
					bitMasks[id] = 0;
				}
				bitMasks[id] |= bit;
			}
			bit = bit << 1;
		}

		ready = true;
	}

	BitMask getBitMask(const string& id) {
		if (not ready) {
			throw runtime_error("bit masks must be generated first");
		}

		if (bitMasks.count(id) == 0) {
			return 0;
		}
		else {
			return bitMasks[id];
		}
	}

	const dict<unordered_set<string>>& getGroups() const {
		return registry;
	}

	dict<unordered_set<string>>& getGroups(){
		return registry;
	}

	void clear() {
		ready = false;

		registry.clear();
		bitMasks.clear();
	}

};