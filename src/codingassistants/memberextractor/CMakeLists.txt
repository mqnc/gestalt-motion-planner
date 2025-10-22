cmake_minimum_required(VERSION 3.17)
project(MemberExtractor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
link_libraries(Threads::Threads)

add_executable(memberextractor
	memberextractor.cpp
)

function(extract_members
	TARGET
	CLASS_NAME
	TARGET_FILE
	JSON_DISPATCH_FILE
	# and SOURCE_FILE_LIST as ARGN
)
	get_target_property(MEMBEREXTRACTOR_DIR memberextractor BINARY_DIR)
	add_custom_command(
		OUTPUT ${TARGET_FILE} ${JSON_DISPATCH_FILE}
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		COMMAND
			${MEMBEREXTRACTOR_DIR}/memberextractor
			${CMAKE_CURRENT_FUNCTION_LIST_DIR}/memberdefinitions.peg
			${TARGET_FILE}
			${JSON_DISPATCH_FILE}
			${CLASS_NAME}
			${ARGN}
		DEPENDS
			memberextractor
			${CMAKE_CURRENT_FUNCTION_LIST_DIR}/memberdefinitions.peg
			${ARGN}
		VERBATIM
	)

	add_custom_target(${TARGET} DEPENDS
		memberextractor
		${ARGN}
		${TARGET_FILE}
		${JSON_DISPATCH_FILE}
	)
endfunction()
