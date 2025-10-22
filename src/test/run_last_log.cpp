
#include "api.h"
#include "utils/timer.h"
#include <iostream>

int main(int argc, char const* argv[]) {
	Timer t;
	#include "../../last_run.log.cpp"
	std::cout << t << "\n";
	return 0;
}