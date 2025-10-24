
#pragma once

#include <charconv>

#include "api.h"
#include "planner_headers.h"

class Log {
public:
	struct Raw { string s; };

private:
	string file;
	stringstream buffer;
	int stackDepth = 0;

	void append() {}

	void append(Raw arg) {
		buffer << arg.s;
	}

	void append(string arg) {
		buffer << '"' << arg << '"';
	}

	void append(const char* arg) {
		buffer << '"' << arg << '"';
	}

	template <typename T, std::enable_if_t<
		std::is_same<T, bool>::value, int
	> = 0>
		void append(T arg) {
		buffer << arg ? "true" : "false";
	}

	void append(int arg) {
		buffer << arg;
	}

	void append(size_t arg) {
		buffer << arg;
	}

	void append(double arg) {
		if (std::isnan(arg)) {
			buffer << "std::nan(\"\")";
		}
		else {
			#ifdef __APPLE__
				// to_chars requires a very recent compiler
				buffer << arg;
			#else
				// this guarantees that the output represents the exact same double
				std::array<char, 64> chars;
				auto result = std::to_chars(chars.data(), chars.data() + chars.size(), arg);
				buffer << std::string(chars.data(), result.ptr - chars.data());
			#endif
		};
	}

	template<typename T>
	void append(vector<T> arg) {
		bool first = true;
		buffer << "{";
		for (const auto& item : arg) {
			if (first) {
				first = false;
			}
			else {
				buffer << ", ";
			}
			append(item);
		}
		buffer << "}";
	}

	template<typename T>
	void append(valarray<T> arg) {
		bool first = true;
		buffer << "{";
		for (const auto& item : arg) {
			if (first) {
				first = false;
			}
			else {
				buffer << ", ";
			}
			append(item);
		}
		buffer << "}";
	}

	template<typename K, typename V>
	void append(std::map<K, V> arg) {
		bool first = true;
		buffer << "{";
		for (const auto& item : arg) {
			if (first) {
				first = false;
			}
			else {
				buffer << ", ";
			}
			buffer << "{";
			append(item.first);
			buffer << ", ";
			append(item.second);
			buffer << "}";
		}
		buffer << "}";
	}

	void append(Pose pose) {
		buffer << "Pose{";
		append(pose.x);
		buffer << ", ";
		append(pose.y);
		buffer << ", ";
		append(pose.z);
		buffer << ", ";
		append(pose.qx);
		buffer << ", ";
		append(pose.qy);
		buffer << ", ";
		append(pose.qz);
		buffer << ", ";
		append(pose.qw);
		buffer << "}";
	}

	template<typename T, typename ... Ts>
	void append(T first, Ts ... tail) {
		append(first);
		buffer << ", ";
		append(tail...);
	}

public:

	Log(string file = "") :file{ file } {
		if (isLogging()) {
			str::save(file, "");
		}
	}

	bool isLogging(){
		return file != "";
	}

	template<typename ... Ts>
	auto log(Raw s) {
		if (isLogging()) {
			append(s);
		}
		stackDepth++;
		return OnScopeExit([this]() {
			this->stackDepth--;
		});
	}

	template<typename ... Ts>
	auto log(string call, Ts ... args) {
		if (isLogging()) {
			for (size_t i = 0; i < stackDepth; i++) {
				buffer << "//\t";
			}
			buffer << call << "(";
			append(args...);
			buffer << ");\n";
			str::append(file, buffer.str());
			buffer.str("");
		}
		stackDepth++;
		return OnScopeExit([this, call]() {
			this->stackDepth--;
			if (isLogging()) {
				for (size_t i = 0; i < this->stackDepth; i++) {
					this->buffer << "//\t";
				}
				this->buffer << "// " << call << " done.\n";
				str::append(this->file, this->buffer.str());
				this->buffer.str("");
			}
		});
	}
};
