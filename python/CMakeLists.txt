cmake_minimum_required(VERSION 3.17)
project(GestaltPlannerPythonBindings)

add_subdirectory(../src/planner ${CMAKE_CURRENT_BINARY_DIR}/../src/planner)
add_subdirectory(../src/codingassistants/pybindgenerator
	${CMAKE_CURRENT_BINARY_DIR}/../src/codingassistants/pybindgenerator)

add_subdirectory(pybind11)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

include(../src/thirdparty/defs.cmake)

include_directories(
	pybind11/include
	../src/planner
	../src/planner/utils
	../src/thirdparty
	../src/thirdparty/ruckig/include
	../src/thirdparty/bullet
	../src/thirdparty/optim/include
	../src/thirdparty/yamlcpp/include
)

generate_python_bindings(bindings
${CMAKE_CURRENT_SOURCE_DIR}/bindings.cpp
	../src/planner/api.h
	GestaltPlanner
	Pose
	PoseUpdate
	Collision
	PlannerParamInfo
	PlannerInfo
)

pybind11_add_module(PyGestaltPlanner ${CMAKE_CURRENT_SOURCE_DIR}/bindings.cpp)
target_link_libraries(PyGestaltPlanner PRIVATE gestaltplanner)

install(TARGETS PyGestaltPlanner LIBRARY DESTINATION .)
