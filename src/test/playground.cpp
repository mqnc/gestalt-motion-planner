#include <common.h>

#include <multi_index_container.hpp>
#include <multi_index/hashed_index.hpp>
#include <multi_index/member.hpp>
#include <multi_index/sequenced_index.hpp>
#include <multi_index/ordered_index.hpp>

#include <unordered_map>

using namespace multi_index;

struct Person {
	string name;
};

// template<auto PM>
// using Registry = multi_index_container <
// 	typename type_from_member<decltype(PM)>::class_type,
// 	indexed_by<
// 	hashed_unique<
// 	member<
// 	typename type_from_member<decltype(PM)>::class_type,
// 	typename type_from_member<decltype(PM)>::member_type, PM
// 	>
// 	>
// 	>
// >;

void frobnicate(const string& name) {
	volatile auto copy = name;
}


int main() {

	// Registry people;

	// size_t n = 100000000;
	// size_t N = 1;

	// std::cout << "start" << "\n";

	// auto startTime = std::chrono::steady_clock::now();


	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < n; i++) {
	// 	people.insert({ std::to_string(i) });
	// }
	// double elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "inserted in " << elapsed << "s\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (const auto& person : people.get<0>()) {
	// 		frobnicate(person.name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "iteratively frobnicated hashed in " << elapsed << "s\n";


	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (const auto& person : people.get<1>()) {
	// 		frobnicate(person.name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "iteratively frobnicated ordered in " << elapsed << "s\n";


	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (size_t i = 0; i < n; i++) {
	// 		frobnicate(people.get<0>().find(std::to_string(i))->name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "randomly frobnicated hashed in " << elapsed << "s\n";


	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (size_t i = 0; i < n; i++) {
	// 		frobnicate(people.get<1>().find(std::to_string(i))->name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "randomly frobnicated ordered in " << elapsed << "s\n";


	// std::unordered_map<string, Person> stdpeople;

	// std::cout << "start" << "\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < n; i++) {
	// 	auto s = std::to_string(i);
	// 	stdpeople.insert({ s, { s } });
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "inserted in " << elapsed << "s\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (const auto& [name, person] : stdpeople) {
	// 		frobnicate(person.name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "std iteratively frobnicated hashed in " << elapsed << "s\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (size_t i = 0; i < n; i++) {
	// 		frobnicate(stdpeople.at(std::to_string(i)).name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "std randomly frobnicated hashed in " << elapsed << "s\n";



	// std::map<string, Person> stdpeople2;

	// std::cout << "start" << "\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < n; i++) {
	// 	auto s = std::to_string(i);
	// 	stdpeople2.insert({ s, { s } });
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "inserted in " << elapsed << "s\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (const auto& [name, person] : stdpeople2) {
	// 		frobnicate(person.name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "std iteratively frobnicated ordered in " << elapsed << "s\n";

	// startTime = std::chrono::steady_clock::now();
	// for (size_t i = 0; i < N; i++) {
	// 	for (size_t i = 0; i < n; i++) {
	// 		frobnicate(stdpeople2.at(std::to_string(i)).name);
	// 	}
	// }
	// elapsed = std::chrono::duration_cast<std::chrono::milliseconds>
	// 	(std::chrono::steady_clock::now() - startTime).count() / 1000.0;
	// std::cout << "std randomly frobnicated ordered in " << elapsed << "s\n";











	return 0;
}

// 


// 	BulletLink(const BulletLink& other) :
// 		id{ other.id },
// 		collisionShape{ other.collisionShape },
// 		visualShape{ other.visualShape }
// 	{
// 		bulletObject = make_unique<btCollisionObject>();
// 		bulletObject.setCollisionShape(
// 			collisionShape->getBulletShape());
// 		parent = other.parent;
// 		children = other.children;
// 		collisionBitMask = other.collisionBitMask;
// 	}

// 	BulletLink(const BulletLinkTemplate& tmpl) :
// 		id{ tmpl.id },
// 		collisionShape{ tmpl.collisionShape },
// 		visualShape{ tmpl.visualShape }
// 	{
// 		bulletObject = make_unique<btCollisionObject>();
// 		bulletObject.setCollisionShape(
// 			collisionShape->getBulletShape());
// 	}
// };






