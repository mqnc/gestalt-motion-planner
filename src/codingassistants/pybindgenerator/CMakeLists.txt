cmake_minimum_required(VERSION 3.17)
project(PyBindGenerator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

add_executable(pybindgenerator
	pybindgenerator.cpp
)

function(generate_python_bindings
	TARGET
	TARGET_FILE
	INPUT_HEADER
	# and CLASS_NAME_LIST as ARGN
)
	message(called from ${CMAKE_CURRENT_LIST_FILE})
	get_target_property(PYBINDGENERATOR_DIR pybindgenerator BINARY_DIR)
	add_custom_command(
		OUTPUT ${TARGET_FILE}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMAND
			g++ -E ${INPUT_HEADER}
			| ${PYBINDGENERATOR_DIR}/pybindgenerator
			${CMAKE_CURRENT_FUNCTION_LIST_DIR}/classdeclarations.peg
			${TARGET_FILE}
			${ARGN}
		DEPENDS
			pybindgenerator
			${CMAKE_CURRENT_FUNCTION_LIST_DIR}/classdeclarations.peg
		VERBATIM
	)

	add_custom_target(${TARGET} DEPENDS
		pybindgenerator
		${INPUT_HEADER}
		${TARGET_FILE}
	)
endfunction()
