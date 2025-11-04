cmake_minimum_required(VERSION 3.10.2)

project(common)

#是x86还是arm下的
option (FOR_X86 "Use as Library" OFF)

if(FOR_X86)
    set(ENV{PATH} "/opt/mine/syzkbuildenv_x86/")    # 定义系统环境变量,只在cmake进程中有
    include_directories(/opt/mine/syzkbuildenv_x86/include)
endif()
MESSAGE(STATUS "FOR_X86        : ${FOR_X86}")

include_directories(./include)
include_directories(./ddsType/include)

#生成动态库
set(LIB_OUT_PATH ../output) #设置库输出目录

if(FOR_X86) #设置库输出路径
    set(LIBRARY_OUTPUT_PATH ${LIB_OUT_PATH}/lib/x86)
else()
    set(LIBRARY_OUTPUT_PATH ${LIB_OUT_PATH}/lib)
endif()

set(INCLUDE_OUTPUT_PATH ${LIB_OUT_PATH}) #设置头文件输出路径

#包含comm中源码文件
aux_source_directory(src SRC_COMMON)
#包含comm中dds消息源码文件
aux_source_directory(ddsType/src SRC_MSG)

add_library(common_share SHARED ${SRC_COMMON} ${SRC_MSG}) #生成动态库
#add_library(common_static STATIC ${SRC_COMMON} ${SRC_MSG}) #生成静态库


file(GLOB COMMON_HEADER ddsType/include include) #记录头文件目录
file(COPY ${COMMON_HEADER} DESTINATION ${INCLUDE_OUTPUT_PATH}) #复制头文件到输出目录

