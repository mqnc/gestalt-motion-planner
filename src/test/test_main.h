
#include "gtest/gtest.h"

#ifdef DEBUGGER // prevent gtest from catching exceptions so the debugger will halt
#define TEST_MAIN \
int main(int argc, char** argv) { \
	std::vector<char*> args(argv, argv + argc); \
	args.push_back((char*)"--gtest_catch_exceptions=0"); \
	int argc1 = argc + 1; \
	::testing::InitGoogleTest(&argc1, args.data()); \
	return RUN_ALL_TESTS(); \
}
#else
#define TEST_MAIN \
int main(int argc, char **argv) { \
	::testing::InitGoogleTest(&argc, argv); \
	return RUN_ALL_TESTS(); \
}
#endif

