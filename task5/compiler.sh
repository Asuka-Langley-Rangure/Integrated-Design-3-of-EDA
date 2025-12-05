#!/usr/bin/env bash
# ------------------------------------------------------------
# task5 一键构建 + 自动清理 + 移动 + 执行
# ------------------------------------------------------------
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${PROJECT_ROOT}/build"
BUILD_TYPE="Release"   # 默认构建类型，可改成 Debug

echo "📁 构建类型: ${BUILD_TYPE}"

# ------------------------------------------------------------
# 1️⃣ 删除旧 build 文件夹
# ------------------------------------------------------------
if [[ -d "${BUILD_DIR}" ]]; then
  echo "🧹 检测到旧的 build 目录，正在删除..."
  rm -rf "${BUILD_DIR}"
  echo "✅ 已删除旧 build 目录"
fi

# ------------------------------------------------------------
# 2️⃣ 环境检查
# ------------------------------------------------------------
need_cmd() { command -v "$1" >/dev/null 2>&1 || { echo "❌ 缺少命令：$1"; exit 1; }; }
need_cmd cmake
need_cmd make || true   # 有些系统用 ninja 也行，这里不强制

# ------------------------------------------------------------
# 3️⃣ 重新配置并构建
# ------------------------------------------------------------
echo "🔧 配置 CMake..."
cmake -S "${PROJECT_ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

JOBS="$(nproc 2>/dev/null || echo 4)"
echo "🏗️  开始编译（并行 ${JOBS}）..."
cmake --build "${BUILD_DIR}" -- -j"${JOBS}"

# ------------------------------------------------------------
# 4️⃣ 定位并移动可执行文件
# ------------------------------------------------------------
BIN_NAME="placementDesigner"
BIN_PATH="$(find "${BUILD_DIR}" -type f -executable -name "${BIN_NAME}" | head -n1 || true)"
TARGET_BIN="${PROJECT_ROOT}/${BIN_NAME}"

if [[ -z "${BIN_PATH}" ]]; then
  echo "❌ 未找到可执行文件 ${BIN_NAME}"
  exit 1
fi

echo "📦 已找到编译产物：${BIN_PATH}"
mv -f "${BIN_PATH}" "${TARGET_BIN}"
echo "🚚 已移动到根目录: ${TARGET_BIN}"

# 拷贝 compile_commands.json（方便 VSCode/clangd 做代码索引）
if [[ -f "${BUILD_DIR}/compile_commands.json" ]]; then
  cp -f "${BUILD_DIR}/compile_commands.json" "${PROJECT_ROOT}/compile_commands.json"
fi

# ------------------------------------------------------------
# 5️⃣ 自动执行
# ------------------------------------------------------------
echo "🚀 正在运行程序..."
"${TARGET_BIN}">"test.txt"
EXIT_CODE=$?

echo "🏁 程序已退出，状态码: ${EXIT_CODE}"
exit ${EXIT_CODE}
