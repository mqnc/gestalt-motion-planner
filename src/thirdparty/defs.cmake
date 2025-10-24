
set( BULLET_DEFS
	PUBLIC BT_EULER_DEFAULT_ZYX
	PUBLIC BT_USE_DOUBLE_PRECISION
	#PUBLIC BT_DEBUG_MEMORY_ALLOCATIONS
)
set( OPTIM_DEFS
	PUBLIC OPTIM_ENABLE_EIGEN_WRAPPERS
)
add_compile_definitions(
	EIGEN_MPL2_ONLY
)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()
find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem program_options)
# find_package(Boost 1.58 QUIET REQUIRED COMPONENTS serialization filesystem system program_options)
include_directories(SYSTEM ${Boost_INCLUDE_DIR})
set( OMPL_DEPENDS
	${Boost_SERIALIZATION_LIBRARY}
	${Boost_FILESYSTEM_LIBRARY}
#	${Boost_SYSTEM_LIBRARY} # listed as a requirement in OMPL but seems to work without it
	${CMAKE_THREAD_LIBS_INIT}
)

set( THIRD_PARTY_DEFINITIONS
	${BULLET_DEFS}
	${OPTIM_DEFS}
	EIGEN_MPL2_ONLY
)

set( THIRD_PARTY_DEPENDENCIES
	${OMPL_DEPENDS}
)
