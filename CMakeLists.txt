
cmake_minimum_required(VERSION 3.4.4)

if(NOT CMAKE_VERSION VERSION_LESS 3.0)
    cmake_policy(SET CMP0048 NEW)
endif()

set(CMAKE_CXX_FLAGS   "-std=c++11 -g")

set(CMAKE_SYSTEM_NAME "Linux")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY bin)
project(syzk-remotectl)

add_definitions("-Wall" "-rdynamic")

# message("===${CMAKE_SOURCE_DIR}")

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/src)

file(GLOB SRC_LIST
	"include/yaml.h"
	"include/ddsType/*.h"
	"include/ddsType/*.cxx"
	"include/*.hpp"
    	"src/common/*.cpp"
        "src/common/*.hpp"
        "src/protocol/*.cpp"
        "src/protocol/*.hpp"
         "src/*.cpp"
    )

add_executable(${PROJECT_NAME} ${SRC_LIST})

target_link_libraries(${PROJECT_NAME}
    sany_utils
    daemon_sdk	
	zkbs   
	FailureReporter
	yaml-cpp
	fastrtps
	fastcdr
	pthread
	rt
	-ldl
)

#file(COPY ${CMAKE_SOURCE_DIR}/src/protocol/remote.ini DESTINATION ${CMAKE_BINARY_DIR}/config)

# #增加设备型号宏
# # add_definitions(-DCONFIG_ELEC_CAR)
