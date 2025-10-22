
#pragma once

#include <iostream>
#include <bitset>
#include <climits>

template<class F>
class OnScopeExit {
	F action;
public:
	OnScopeExit(F action) :action{ action } {}
	~OnScopeExit() { action(); }
};

struct PairHash {
	template <class T1, class T2>
	size_t operator () (const std::pair<T1, T2>& p) const {
		auto h1 = std::hash<T1>{}(p.first);
		auto h2 = std::hash<T2>{}(p.second);
		return h1 ^ (2 * h2);
	}
};

/* to do
#ifdef __SIZEOF_INT128__
using BitMask = unsigned __int128;
#else
[[deprecated("__int128 not supported, only 64 bit masks will be allowed")]]
void warning_int128_not_supported();
using BitMask = uint64_t;
#endif
*/

using BitMask = int;

inline const size_t MAX_BIT = sizeof(BitMask) * 8 - 1;

template<typename T>
struct BinaryForm {
	BinaryForm(const T& v) : _bs(v) {}
	const std::bitset<sizeof(T)* CHAR_BIT> _bs;
};

template<typename T>
inline std::ostream& operator<<(std::ostream& os, const BinaryForm<T>& bf) {
	return os << bf._bs;
}

template <typename...> struct WhichType;

inline void makeDir(std::string dir) {
	if (std::filesystem::exists(dir) && !std::filesystem::is_directory(dir)) {
		throw runtime_error(string() + "a file named '" + dir + "' exists");
	}
	if (!std::filesystem::exists(dir)) {
		std::filesystem::create_directory(dir);
	}
}
