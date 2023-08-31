#!/bin/bash

project_root="/home/ubuntu/chaibo/Remote_auto"
TOOLCHAIN="/opt/fsl-imx-wayland/5.4-zeus/environment-setup-aarch64-poky-linux"

show_help()
{
    cat <<EOF
Syntax: ./nl [command]

Available commands:
    all       Compile the whole project
    clean     Clear all generated files

EOF
}

get_project_root()
{
    local dir=`pwd`

    # if [ -d "$dir/src/vehicle" -a -d "$dir/src/excavator" ]; then
    #     project_root="$dir"
    # else
    #     echo "Error:请在工程根目录下执行编译命令!"
    #     exit -1
    # fi
}

configure_compile_toolchain()
{
    if [ ! -f "$TOOLCHAIN" ]; then
        echo "Error:请确认设置的编译工具链路径是否正确!"
        exit -1
    fi
    
    source $TOOLCHAIN
    unset LD_LIBRARY_PATH   
}

build_all()
{
    # if [ ! -d ${project_root}/build ]; then
    #     mkdir -p ${project_root}/build
    # fi

    cd ${project_root}/build
    cmake .. 
    make

    # if [ ! -d ${project_root}/output ]; then
    #     mkdir -p ${project_root}/output
    # fi

 #   cp -rf ${project_root}/build/src/vehicle/v2x_vehicle     ${project_root}/output
 #   cp -rf ${project_root}/build/src/excavator/v2x_excavator ${project_root}/output
 #   cp -rf ${project_root}/build/test/v2x_test               ${project_root}/output
 #   cp -rf ${project_root}/config/*.ini                      ${project_root}/output
 #   cp -rf ${project_root}/external/sany_uilts/target/lib/libsany_utils.so  ${project_root}/output
}

all()
{
    configure_compile_toolchain

    build_all
}

clean()
{
    rm -rf ${project_root}/build/*
    rm -rf ${project_root}/output/*
    echo "clean files ok."
}

get_project_root

case "$1" in
    all|clean)
        $*
    ;;
    *)
        show_help
    ;;
esac
