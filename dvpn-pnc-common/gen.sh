#!/bin/bash

if [ -d "idl" ]; 
then
	cd idl
        for subIdl in `ls *idl` #注意此处这是两个反引号，表示运行系统命令
        do
            echo $subIdl
            #安装头文件
            mine gen $subIdl ../ddsType
        done
	mv ../ddsType/*.cxx ../ddsType/src
	mv ../ddsType/*.h ../ddsType/include
else
	echo "idl find fail"
fi



