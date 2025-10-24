cmake_minimum_required(VERSION 3.10)
project(GestaltPlanner)

add_subdirectory(../thirdparty
	${CMAKE_CURRENT_BINARY_DIR}/../thirdparty)
add_subdirectory(../codingassistants/memberextractor
	${CMAKE_CURRENT_BINARY_DIR}/../codingassistants/memberextractor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)
# find_package(Ceres REQUIRED)

###############
# definitions #
###############

set( GESTALT_PLANNER_DEFS PUBLIC
	COLLISION_BETWEEN_INFLATED_BOXES_CONSIDERS_ROUNDED_EDGES
	#PROFILING
)

###########
# sources #
###########

# not using glob here so cmake detects changes
set(PLANNER_SOURCES
	collision/shapes.cpp
	collision/multibody.cpp
	collision/collisionrobot.cpp
	collision/collisionchecker.cpp
	interpolator/interpolator.cpp
	#kinematics/inverse_kinematics.cpp
	urdf/urdf.cpp
	#encapsulator/encapsulate.cpp
	#render/htmlconverter.cpp
	#export/collisionprobe.cpp
	render/meshgenerator.cpp
	render/wraphtml.cpp
	pathfinder/findpath.cpp

	# find main/ -name "*.cpp"
	main/bttransformfrompose.cpp
	main/cache_file.cpp
	main/check_clearance.cpp
	main/find_collisions.cpp
	main/check_kinematic_feasibility.cpp
	main/collision_ignore_group.cpp
	main/command_log_comment.cpp
	main/ctordtor.cpp
	main/export.cpp
	main/import.cpp
	main/render.cpp
	main/export_state.cpp
	main/time_parameterize_path.cpp
	main/interpolate.cpp
	main/plan_path.cpp
	main/plan_multistep_path.cpp
	main/plannerstate.cpp
	main/remove.cpp
	main/reset.cpp
	main/select_joints.cpp
	main/set_base_pose.cpp
	main/set_joint_positions.cpp
	main/set_safety_margin.cpp
	main/simplify_path.cpp
	main/smoothen_path.cpp
	main/spawn.cpp
	main/spawn_world.cpp
	main/tighten_path.cpp
	main/json_call.cpp
	# main/inverse_kinematics.cpp
	# main/stringify.cpp
)

#########################
# third party libraries #
#########################

set(PLANNER_LIBS
	bullet_embedded
	ompl_embedded
	# optim_embedded
	quickhull_embedded
	ruckig_embedded
	tinyxml2_embedded
	yamlcpp_embedded
)

include(../thirdparty/defs.cmake)

include_directories(
	.
	utils
	../thirdparty
	../thirdparty/ruckig/include
	../thirdparty/bullet
	../thirdparty/optim/include
	../thirdparty/yamlcpp/include
	../thirdparty/multi_index/include
	../thirdparty/brigand/include
)

#######################
# extract api members #
#######################

extract_members(plannermethods
	GestaltPlanner
	${CMAKE_CURRENT_LIST_DIR}/plannermethods.inc
	${CMAKE_CURRENT_LIST_DIR}/main/jsondispatch.inc
	${PLANNER_SOURCES}
)

#################
# build planner #
#################

link_libraries(${PLANNER_LIBS} ${THIRD_PARTY_DEPENDENCIES})

add_library(gestaltplanner ${PLANNER_SOURCES})
# precompiled headers cause an additional planner rebuild in docker
# target_precompile_headers(gestaltplanner PRIVATE utils/common.h)
target_compile_definitions(gestaltplanner PUBLIC
	${GESTALT_PLANNER_DEFS} ${THIRD_PARTY_DEFINITIONS})
add_dependencies(gestaltplanner plannermethods)

# target_link_libraries (gestaltplanner Ceres::ceres)
