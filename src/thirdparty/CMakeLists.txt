cmake_minimum_required(VERSION 3.10)
project(GestaltPlannerThirdPartyLibs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wno-stringop-overflow")
endif()

include(defs.cmake)

include_directories(
	.
	ruckig/include
	bullet
	optim/include
	yamlcpp/include
)

##########
# bullet #
##########

add_library(bullet_embedded
	bullet/btLinearMathAll.cpp
	bullet/btBulletCollisionAll.cpp
)
target_compile_definitions(bullet_embedded ${BULLET_DEFS})

########
# ompl #
########

configure_file(
	"ompl/config.h.in"
	"ompl/config.h"
)
# file(GLOB_RECURSE OMPL_SOURCE
# 	ompl/*.cpp
# )
# add_library(ompl_embedded
# 	${OMPL_SOURCE}
# )
add_library(ompl_embedded
	ompl/base/goals/src/GoalRegion.cpp
	ompl/base/goals/src/GoalState.cpp
	ompl/base/goals/src/GoalStates.cpp
	ompl/base/objectives/src/MaximizeMinClearanceObjective.cpp
	ompl/base/objectives/src/MechanicalWorkOptimizationObjective.cpp
	ompl/base/objectives/src/MinimaxObjective.cpp
	ompl/base/objectives/src/PathLengthOptimizationObjective.cpp
	ompl/base/samplers/deterministic/src/HaltonSequence.cpp
	ompl/base/samplers/informed/src/OrderedInfSampler.cpp
	ompl/base/samplers/informed/src/PathLengthDirectInfSampler.cpp
	ompl/base/samplers/informed/src/RejectionInfSampler.cpp
	ompl/base/samplers/src/DeterministicStateSampler.cpp
	ompl/base/samplers/src/InformedStateSampler.cpp
	ompl/base/samplers/src/UniformValidStateSampler.cpp
	ompl/base/spaces/constraint/src/ConstrainedStateSpace.cpp
	ompl/base/spaces/src/DubinsStateSpace.cpp
	ompl/base/spaces/src/RealVectorBounds.cpp
	ompl/base/spaces/src/RealVectorStateProjections.cpp
	ompl/base/spaces/src/RealVectorStateSpace.cpp
	ompl/base/spaces/src/ReedsSheppStateSpace.cpp
	ompl/base/spaces/src/SE2StateSpace.cpp
	ompl/base/spaces/src/WrapperStateSpace.cpp
	ompl/base/src/Constraint.cpp
	ompl/base/src/Cost.cpp
	ompl/base/src/DiscreteMotionValidator.cpp
	ompl/base/src/GenericParam.cpp
	ompl/base/src/Goal.cpp
	ompl/base/src/OptimizationObjective.cpp
	ompl/base/src/Planner.cpp
	ompl/base/src/PlannerData.cpp
	ompl/base/src/PlannerTerminationCondition.cpp
	ompl/base/src/PrecomputedStateSampler.cpp
	ompl/base/src/ProblemDefinition.cpp
	ompl/base/src/ProjectionEvaluator.cpp
	ompl/base/src/SpaceInformation.cpp
	ompl/base/src/StateSampler.cpp
	ompl/base/src/StateSpace.cpp
	ompl/base/src/StateStorage.cpp
	ompl/base/src/ValidStateSampler.cpp
	ompl/base/terminationconditions/src/IterationTerminationCondition.cpp
	ompl/control/planners/kpiece/src/KPIECE1.cpp
	ompl/control/planners/rrt/src/RRT.cpp
	ompl/control/spaces/src/DiscreteControlSpace.cpp
	ompl/control/src/ControlSampler.cpp
	ompl/control/src/ControlSpace.cpp
	ompl/control/src/PathControl.cpp
	ompl/control/src/SimpleDirectedControlSampler.cpp
	ompl/control/src/SpaceInformation.cpp
	ompl/geometric/planners/est/src/BiEST.cpp
	ompl/geometric/planners/est/src/EST.cpp
	ompl/geometric/planners/est/src/ProjEST.cpp
	ompl/geometric/planners/fmt/src/BFMT.cpp
	ompl/geometric/planners/fmt/src/FMT.cpp
	ompl/geometric/planners/kpiece/src/BKPIECE1.cpp
	ompl/geometric/planners/kpiece/src/KPIECE1.cpp
	ompl/geometric/planners/kpiece/src/LBKPIECE1.cpp
	ompl/geometric/planners/pdst/src/PDST.cpp
	ompl/geometric/planners/prm/src/LazyPRM.cpp
	ompl/geometric/planners/prm/src/LazyPRMstar.cpp
	ompl/geometric/planners/prm/src/PRM.cpp
	ompl/geometric/planners/prm/src/PRMstar.cpp
	ompl/geometric/planners/prm/src/SPARS.cpp
	ompl/geometric/planners/prm/src/SPARStwo.cpp
	ompl/geometric/planners/rrt/src/InformedRRTstar.cpp
	ompl/geometric/planners/rrt/src/LBTRRT.cpp
	ompl/geometric/planners/rrt/src/LazyLBTRRT.cpp
	ompl/geometric/planners/rrt/src/LazyRRT.cpp
	ompl/geometric/planners/rrt/src/RRT.cpp
	ompl/geometric/planners/rrt/src/RRTConnect.cpp
	ompl/geometric/planners/rrt/src/RRTXstatic.cpp
	ompl/geometric/planners/rrt/src/RRTsharp.cpp
	ompl/geometric/planners/rrt/src/RRTstar.cpp
	ompl/geometric/planners/rrt/src/SORRTstar.cpp
	ompl/geometric/planners/rrt/src/TRRT.cpp
	ompl/geometric/planners/sbl/src/SBL.cpp
	ompl/geometric/planners/sst/src/SST.cpp
	ompl/geometric/planners/stride/src/STRIDE.cpp
	ompl/geometric/src/PathGeometric.cpp
	ompl/geometric/src/PathSimplifier.cpp
	ompl/geometric/src/SimpleSetup.cpp
	ompl/tools/config/src/SelfConfig.cpp
	ompl/util/src/Console.cpp
	ompl/util/src/GeometricEquations.cpp
	ompl/util/src/ProlateHyperspheroid.cpp
	ompl/util/src/RandomNumbers.cpp
	ompl/util/src/String.cpp
)
target_link_libraries(ompl_embedded ${OMPL_DEPENDS})

#########
# optim #
#########

# this was needed only for nelder mead which was then manually implemented, but we keep it in, might need some optimization from it
# add_library(optim_embedded
# 	optim/src/unconstrained/nm.cpp
# )
# target_compile_definitions(optim_embedded ${OPTIM_DEFS})

############
# tinyxml2 #
############

add_library(tinyxml2_embedded
	tinyxml2/tinyxml2.cpp
)

#############
# quickhull #
#############

add_library(quickhull_embedded
	quickhull/QuickHull.cpp
)

##########
# ruckig #
##########

# find ruckig/src/ -name "*.cpp"
# except bindings
add_library(ruckig_embedded
	ruckig/src/ruckig/velocity_third_step2.cpp
	ruckig/src/ruckig/velocity_second_step2.cpp
	ruckig/src/ruckig/velocity_second_step1.cpp
	ruckig/src/ruckig/position_second_step2.cpp
	ruckig/src/ruckig/position_first_step1.cpp
	ruckig/src/ruckig/position_third_step1.cpp
	ruckig/src/ruckig/position_third_step2.cpp
	#ruckig/src/ruckig/wasm.cpp
	ruckig/src/ruckig/brake.cpp
	ruckig/src/ruckig/velocity_third_step1.cpp
	#ruckig/src/ruckig/python.cpp
	ruckig/src/ruckig/position_second_step1.cpp
	ruckig/src/ruckig/position_first_step2.cpp
	#ruckig/src/ruckig/cloud_client.cpp
)

###########
# yamlcpp #
###########

# file(GLOB_RECURSE YAMLCPP_SOURCE
# 	yamlcpp/src/*.cpp
# )
# add_library(yamlcpp_embedded
# 	${YAMLCPP_SOURCE}
# )
add_library(yamlcpp_embedded
	yamlcpp/src/depthguard.cpp
	yamlcpp/src/directives.cpp
	yamlcpp/src/exceptions.cpp
	yamlcpp/src/exp.cpp
	yamlcpp/src/memory.cpp
	yamlcpp/src/node_data.cpp
	yamlcpp/src/nodebuilder.cpp
	yamlcpp/src/null.cpp
	yamlcpp/src/parse.cpp
	yamlcpp/src/parser.cpp
	yamlcpp/src/regex_yaml.cpp
	yamlcpp/src/scanner.cpp
	yamlcpp/src/scanscalar.cpp
	yamlcpp/src/scantag.cpp
	yamlcpp/src/scantoken.cpp
	yamlcpp/src/simplekey.cpp
	yamlcpp/src/singledocparser.cpp
	yamlcpp/src/stream.cpp
	yamlcpp/src/tag.cpp
)