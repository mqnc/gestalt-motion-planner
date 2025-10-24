cmake_minimum_required(VERSION 3.10)
project(GestaltPlannerTests)

add_subdirectory(../planner ${CMAKE_CURRENT_BINARY_DIR}/../planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DDEBUGGER")

include(../thirdparty/defs.cmake)

include_directories(
	../planner
	../planner/utils
	../thirdparty
	../thirdparty/ruckig/include
	../thirdparty/bullet
	../thirdparty/optim/include
	../thirdparty/yamlcpp/include
	../thirdparty/brigand/include
	../thirdparty/multi_index/include
)

enable_testing()

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

macro(make_test NAME)
	add_executable(${NAME} ${NAME}.cpp)
	# causes some issues with static asserts
	# target_precompile_headers(${NAME} PUBLIC ../planner/utils/common.h)
	target_link_libraries(${NAME} gestaltplanner GTest::gtest_main)
	target_compile_definitions(${NAME} ${THIRD_PARTY_DEFINITIONS})
	gtest_discover_tests(${NAME})
endmacro()

make_test(test_test)
make_test(test_bullet)
make_test(test_stl)
# make_test(test_encapsulate)
make_test(test_api)
make_test(test_margin)
make_test(test_compound)
make_test(test_json)
make_test(test_yaml)
make_test(test_quickhull)
make_test(test_zip)
# make_test(test_optim_ik)

add_executable(last_log EXCLUDE_FROM_ALL
	run_last_log.cpp
)
target_link_libraries(last_log gestaltplanner)

add_executable(playground EXCLUDE_FROM_ALL
	playground.cpp
)
target_link_libraries(playground bullet_embedded)

add_executable(plannerplayground EXCLUDE_FROM_ALL
	plannerplayground.cpp
)
target_link_libraries(plannerplayground gestaltplanner)
