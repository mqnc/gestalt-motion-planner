
// adapted from https://www.codesynthesis.com/~boris/blog/2012/09/11/emulating-boost-multi-index-with-std-containers/

#include <map>
// #include <tuple>

template <typename T>
struct key_pointer {
	mutable const T* p;
	key_pointer(const T* v = nullptr) : p(v) {}
	bool operator< (const key_pointer& other) const { return *p < *other.p; }
};

template <typename I>
struct map_iterator_adapter : I {
	using value_type = typename I::value_type::second_type;
	using pointer = value_type*;
	using reference = value_type&;

	map_iterator_adapter() {}
	map_iterator_adapter(I i) : I(i) {}

	reference operator*() { return I::operator*().second; }
	pointer operator->() { return &I::operator-> ()->second; }
};

template <typename I>
struct map_const_iterator_adapter : I {
	using value_type = const typename I::value_type::second_type;
	using pointer = value_type*;
	using reference = value_type&;

	map_const_iterator_adapter() {}
	map_const_iterator_adapter(I i) : I(i) {}
	template<typename I2>
	map_const_iterator_adapter(map_iterator_adapter<I2> i) : I(i) {}

	reference operator* () const { return I::operator* ().second; }
	pointer operator-> () const { return &I::operator-> ()->second; }
};

template <
	typename T,
	typename key_ref
>
class Registry {
public:
	using entry_type = T;
	using key_ref_res = std::invoke_result_t<key_ref, const entry_type&>;
	static_assert(std::is_reference_v<key_ref_res>);
	using key_type = std::decay_t<key_ref_res>;
	using map_type = std::map<key_pointer<key_type>, entry_type>;
	using iterator = map_iterator_adapter<typename map_type::iterator>;
	using const_iterator = map_const_iterator_adapter<typename map_type::const_iterator>;

	template<typename V>
	std::pair<iterator, bool> insert_impl(V&& value) {
		auto& temp = key_ref{}(value);

		auto [raw_it, ok] = registry.insert(std::make_pair(
			key_pointer(&temp),
			std::forward<V>(value)
		));
		iterator it(raw_it);
		if (ok) { raw_it->first.p = &key_ref{}(raw_it->second); }
		return std::make_pair(it, ok);
	}

	std::pair<iterator, bool> insert(const entry_type& value) {
		return insert_impl(value);
	}
	std::pair<iterator, bool> insert(entry_type&& value) {
		return insert_impl(std::move(value));
	}

	iterator find(const key_type& key) {
		return registry.find(&key);
	}
	const_iterator find(const key_type& key) const {
		return registry.find(&key);
	}

	entry_type& at(const key_type& key) {
		auto it = find(key);
		if (it == end()) { throw std::range_error("item not found"); }
		return *it;
	}
	const entry_type& at(const key_type& key) const {
		auto it = find(key);
		if (it == end()) { throw std::range_error("item not found"); }
		return *it;
	}

	bool contains(const key_type& key) const {
		return registry.count(&key) > 0;
	}

	iterator begin() { return registry.begin(); }
	const_iterator begin() const { return registry.begin(); }
	iterator end() { return registry.end(); }
	const_iterator end() const { return registry.end(); }

private:
	map_type registry;
};

template <
	typename T,
	typename primary_key_ref,
	typename secondary_key_ref
>
class DualRegistry {
public:
	using entry_type = T;
	using primary_registry_type = Registry<entry_type, primary_key_ref>;
	using primary_key_type = typename primary_registry_type::key_type;
	using iterator = typename primary_registry_type::iterator;
	using const_iterator = typename primary_registry_type::const_iterator;
	using secondary_key_ref_res = std::invoke_result_t<secondary_key_ref, const entry_type&>;
	static_assert(std::is_reference_v<secondary_key_ref_res>);
	using secondary_key_type = std::decay_t<secondary_key_ref_res>;
	using secondary_map_type = std::map<key_pointer<secondary_key_type>, iterator>;

	template<typename V>
	std::pair<iterator, bool> insert_impl(V&& value) {
		auto& temp = secondary_key_ref{}(value);

		auto i2 = secondary_registry.find(&temp);
		if (i2 != secondary_registry.end()) { return std::make_pair(i2->second, false); }

		auto [it, ok] = primary_registry.insert(std::forward<V>(value));

		if (ok) { secondary_registry.insert({ &secondary_key_ref{}(*it), it }); }

		return std::make_pair(it, ok);
	}

	std::pair<iterator, bool> insert(const entry_type& value) {
		return insert_impl(value);
	}
	std::pair<iterator, bool> insert(entry_type&& value) {
		return insert_impl(std::move(value));
	}

	iterator find(primary_key_ref, const primary_key_type& key1) {
		return primary_registry.find(key1);
	}
	const_iterator find(primary_key_ref, const primary_key_type& key1) const {
		return primary_registry.find(key1);
	}

	iterator find(secondary_key_ref, const secondary_key_type& key2) {
		auto i(secondary_registry.find(&key2));
		return i != secondary_registry.end() ? i->second : end();
	}
	const_iterator find(secondary_key_ref, const secondary_key_type& key2) const {
		auto i(secondary_registry.find(&key2));
		return i != secondary_registry.end() ? i->second : end();
	}

	template<typename key_tag, typename K>
	entry_type& at(key_tag tag, const K& key) {
		auto it = find(tag, key);
		if (it == end()) { throw std::range_error("item not found"); }
		return *it;
	}
	template<typename key_tag, typename K>
	const entry_type& at(key_tag tag, const K& key) const {
		auto it = find(tag, key);
		if (it == end()) { throw std::range_error("item not found"); }
		return *it;
	}

	template<typename key_tag, typename K>
	bool contains(key_tag tag, const K& key) const {
		return find(tag, key) != end();
	}

	iterator begin() { return primary_registry.begin(); }
	const_iterator begin() const { return primary_registry.begin(); }
	iterator end() { return primary_registry.end(); }
	const_iterator end() const { return primary_registry.end(); }

private:
	primary_registry_type primary_registry;
	secondary_map_type secondary_registry;
};



// // https://stackoverflow.com/a/42348356/3825996
// template<class In, template<class...>class Map>
// struct map_tuple;
// template<class In, template<class...>class Map>
// using map_tuple_t = typename map_tuple<In, Map>::type;

// template<template<class...>class Z, class...Ts, template<class...>class Map>
// struct map_tuple<Z<Ts...>, Map> {
// 	using type = Z<Map<Ts>...>;
// };


// template <
// 	typename T,
// 	typename primary_key_ref,
// 	typename ... further_key_refs
// >
// class MultiKeyRegistry {
// 	using entry_type = T;

// 	using primary_registry = Registry<entry_type, primary_key_ref>;

// 	using further_key_types = std::tuple<
// 		std::decay_t<std::invoke_result_t<further_key_refs, const entry_type&>>...
// 	>;

// 	using iterator = typename primary_registry::iterator;

// 	// template<typename K>
// 	// struct key_type_to_map_type {
// 	// 	using type = std::map<key_pointer<K>, iterator>;
// 	// };

// 	// using further_map_types = map_tuple_t<further_key_types, key_type_to_map_type>;

// 	using further_map_types = std::tuple<
// 		std::map<
// 		key_pointer<std::decay_t<std::invoke_result_t<further_key_refs, const entry_type&>>>,
// 		iterator
// 		>
// 		...>;

// 	template<typename key_ref, typename reg>
// 	bool check_free(const V& value) {	

// 	template<typename V>
// 	std::pair<iterator, bool> insert_impl(V&& value) {



// 		auto i2 = secondary_registry.find(&((*value).*pointer_to_member2));
// 		if (i2 != secondary_registry.end()) { return std::make_pair(i2->second, false); }

// 		auto [raw_it, ok] = registry.insert({
// 			&((*value).*pointer_to_member1),
// 			std::forward<V>(value)
// 			});
// 		iterator it(raw_it);

// 		if (ok) {
// 			raw_it->first.p = &((**it).*pointer_to_member1);
// 			secondary_registry.insert({ &((**it).*pointer_to_member2), it });
// 		}

// 		return std::make_pair(it, ok);
// 	}


// private:
// 	primary_registry registry;
// 	further_map_types further_registries;
// };

#ifdef TESTS_SNIPPET

#include <cassert>
#include <string>
#include <iostream>
#include <memory>

struct Person {
	const std::string email;
	const std::string name;
	int age;
};

struct Employee : public Person {
	std::string job;
};

struct Movable {
	const int id;
	Movable(int id) :id(id) {};
	Movable(const Movable&) = delete;
	Movable(Movable&&) = default;
};

struct Diva {
	const std::string name;
	const int age;
	Diva(std::string name, int age) :name(name), age(age) {};
	Diva(const Diva&) = delete;
	Diva(Diva&&) = delete;
};

struct by_name {
	const std::string& operator() (const Person& p) {
		return p.name;
	}
};

struct by_p_name {
	const std::string& operator() (const std::unique_ptr<Person>& p) {
		return p->name;
	}
};

struct by_diva_name {
	const std::string& operator() (const std::unique_ptr<Diva>& p) {
		return p->name;
	}
};

struct by_email {
	const std::string& operator() (const Person& p) {
		return p.email;
	}
};

struct by_p_email {
	const std::string& operator() (const std::unique_ptr<Person>& p) {
		return p->email;
	}
};

struct by_diva_age {
	const int& operator() (const std::unique_ptr<Diva>& p) {
		return p->age;
	}
};

int main() {
	Registry<Person, by_name> myRegistry;

	myRegistry.insert({ "one", "oneone", 1 });
	myRegistry.insert({ "two", "twotwo", 2 });
	myRegistry.insert({ "john@doe.com", "John Doe", 29 });
	myRegistry.insert({ "jane@doe.com", "Jane Doe", 27 });

	std::cout << myRegistry.find("John Doe")->age << "\n";
	std::cout << myRegistry.find("Jane Doe")->age << "\n";

	assert(myRegistry.find("John Doe")->age == 29);
	assert(myRegistry.find("Jane Doe")->age == 27);


	Registry<std::unique_ptr<Person>, by_p_name> mypRegistry;

	mypRegistry.insert(std::unique_ptr<Person>(new Person{ "john@doe.com", "John Doe", 29 }));
	mypRegistry.insert(std::unique_ptr<Person>(new Person{ "jane@doe.com", "Jane Doe", 27 }));

	assert(mypRegistry.find("John Doe")->get()->age == 29);
	assert(mypRegistry.find("Jane Doe")->get()->age == 27);


	DualRegistry<Person, by_email, by_name> myDualRegistry;

	myDualRegistry.insert({ "john@doe.com", "John Doe", 29 });
	myDualRegistry.insert({ "jane@doe.com", "Jane Doe", 27 });

	myDualRegistry.find(by_email{}, "jane@doe.com")->age++;
	assert(myDualRegistry.find(by_name{}, "Jane Doe")->age == 28);

	for (auto& p : myDualRegistry) {
		p.age++;
	}

	assert(myDualRegistry.find(by_name{}, "John Doe")->age == 30);
	assert(myDualRegistry.find(by_name{}, "Jane Doe")->age == 29);


	// Registry<&Person::email, Employee> myEmpRegistry;
	// myEmpRegistry.insert({ { "john@doe.com", "John Doe", 29 }, "privatier" });


	// Registry<&Movable::id> myMovRegistry;
	// myMovRegistry.insert(Movable(5));


	Registry<std::unique_ptr<Diva>, by_diva_name> myDivaRegistry;
	myDivaRegistry.insert(std::make_unique<Diva>("Dina", 47));
	assert(myDivaRegistry.at("Dina")->age == 47);

	DualRegistry<std::unique_ptr<Diva>, by_diva_name, by_diva_age> myDivaDualRegistry;
	myDivaDualRegistry.insert(std::make_unique<Diva>("Dina", 47));
	assert(myDivaDualRegistry.at(by_diva_name{}, "Dina")->age == 47);
	assert(myDivaDualRegistry.at(by_diva_age{}, 47)->name == "Dina");


	for(const auto& diva:myDivaRegistry){
		std::cout << diva->age << "\n";
	}

	for(const auto& diva:myDivaDualRegistry){
		std::cout << diva->age << "\n";
	}

	myDivaRegistry.find("Dina");

	std::cout << "done\n";


	// auto pd = std::make_unique<Diva>("Dina", 47);
	// auto ppd = &pd;
	// auto pwpd = Ptr(&pd);
	// pwpd->


	return 0;

}

#endif