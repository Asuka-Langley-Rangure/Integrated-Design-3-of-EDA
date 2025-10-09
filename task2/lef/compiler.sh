#!/bin/bash

# 自动化编译和运行脚本
BUILD_DIR="build"
EXEC_NAME="lef_example"
ERROR_FILE="Error.txt"

# 创建构建目录
if [ ! -d "$BUILD_DIR" ]; then
    mkdir "$BUILD_DIR"
fi

# 进入构建目录并编译
cd "$BUILD_DIR"
cmake ..
make

# 返回上级目录
cd ..
# 移动可执行文件到当前目录
if [ -f "$BUILD_DIR/$EXEC_NAME" ]; then
    mv -f "$BUILD_DIR/$EXEC_NAME" .
fi

echo "--- 运行程序输出 ---"
# 运行程序并将标准错误输出重定向到Error.txt

./$EXEC_NAME "$@" > Out.txt 2> Err.txt

echo $?   # 打印退出码，段错误一般是 139
