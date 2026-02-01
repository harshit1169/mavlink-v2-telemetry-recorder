cmake_minimum_required(VERSION 3.16)
project(mavlink_telemetry_recorder LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SRC_FILES
	src/main.cpp
	src/mavlinkparser.cpp
	src/crc_x25.cpp
)

add_executable(mavlink_telemetry_recorder ${SRC_FILES})

target_include_directories(mavlink_telemetry_recorder PRIVATE
	${CMAKE_CURRENT_SOURCE_DIR}/include
	${CMAKE_CURRENT_SOURCE_DIR}/libs/mavlink_cLib_v2
)

if (MSVC)
	target_compile_options(mavlink_telemetry_recorder PRIVATE /W4 /WX)
else()
	target_compile_options(mavlink_telemetry_recorder PRIVATE -Wall -Wextra -Werror)

	# Suppress warnings only for MAVLink headers
	target_compile_options(mavlink_telemetry_recorder PRIVATE
		-Wno-pedantic
		-Wno-address-of-packed-member
	)
endif()

message(STATUS "Project Configured for aerospace safety critical standards")
