#!/bin/bash
# ============================================================
# 🚀 一键编译与运行脚本：build_run.sh
# 适用于 MyPlacement 项目
# ============================================================

# 1️⃣ 进入脚本所在目录
cd "$(dirname "$0")"

# 2️⃣ 删除旧的 build 文件夹（防止缓存冲突）
if [ -d "build" ]; then
    echo "🧹 删除旧的 build 文件夹..."
    rm -rf build
fi

# 3️⃣ 创建新的 build 目录并进入
echo "📁 创建新的 build 目录..."
mkdir -p build
cd build

# 4️⃣ 运行 CMake（Release 模式）
echo "🔧 运行 CMake 配置 (Release 模式)..."
cmake -DCMAKE_BUILD_TYPE=Release ..

# 检查是否配置成功
if [ $? -ne 0 ]; then
    echo "❌ CMake 配置失败！"
    exit 1
fi

# 5️⃣ 编译项目
echo "🏗️ 开始编译..."
make -j$(nproc)

# 检查编译结果
if [ $? -ne 0 ]; then
    echo "❌ 编译失败！"
    exit 1
fi

# 6️⃣ 将生成的 myplace 移动到项目根目录
if [ -f "myplace" ]; then
    echo "🚚 将可执行文件移动到项目根目录..."
    mv myplace ..
else
    echo "❌ 未找到可执行文件 myplace"
    exit 1
fi

# 7️⃣ 返回项目根目录并运行
cd ..
echo "✅ 编译成功，开始执行..."
./myplace

# 8️⃣ 可选：执行结束后提示
echo "🏁 程序执行完毕！"
