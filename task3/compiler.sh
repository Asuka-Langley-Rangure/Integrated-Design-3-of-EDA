#!/bin/bash

# 自动化编译和运行脚本
BUILD_DIR="build"
EXEC_NAME="myplace"
OUTPUT_FILE="Output.txt"

# 如果存在旧的 build 目录则删除
if [ -d "$BUILD_DIR" ]; then
    echo ">>> 检测到旧的构建目录，正在删除..."
    rm -rf "$BUILD_DIR"
fi

# 重新创建构建目录
mkdir "$BUILD_DIR"

# 进入构建目录并编译
cd "$BUILD_DIR" || exit
echo ">>> 运行 CMake 配置..."
cmake .. || { echo "CMake 配置失败"; exit 1; }

echo ">>> 开始编译..."
make -j$(nproc) || { echo "编译失败"; exit 1; }

# 返回上级目录
cd ..

# 移动可执行文件到当前目录
if [ -f "$BUILD_DIR/$EXEC_NAME" ]; then
    mv -f "$BUILD_DIR/$EXEC_NAME" .
else
    echo "错误：未找到可执行文件 $BUILD_DIR/$EXEC_NAME"
    exit 1
fi

echo "--- 运行程序输出 ---"
# 运行程序并将标准输出重定向到 Output.txt
./$EXEC_NAME > "$OUTPUT_FILE" 2>&1

echo ">>> 程序运行结束，输出已保存到 $OUTPUT_FILE"
