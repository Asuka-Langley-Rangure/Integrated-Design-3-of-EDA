#!/usr/bin/env bash
set -e

PROJECT_ROOT="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$PROJECT_ROOT/build"
TARGET_NAME="myplace"

echo "=============================="
echo "  CLEAN OLD BUILD IF EXISTS..."
echo "=============================="
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"

echo "=============================="
echo "  CMAKE CONFIGURE & BUILD..."
echo "=============================="
cmake -S "$PROJECT_ROOT" -B "$BUILD_DIR" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

cmake --build "$BUILD_DIR" --parallel

# echo "=============================="
# echo "  EXPORT compile_commands.json"
# echo "=============================="
# cp -f "$BUILD_DIR/compile_commands.json" "$PROJECT_ROOT" 2>/dev/null || true

echo "=============================="
echo "  MOVE EXECUTABLE TO ROOT..."
echo "=============================="
if [ -f "$BUILD_DIR/$TARGET_NAME" ]; then
    mv -f "$BUILD_DIR/$TARGET_NAME" "$PROJECT_ROOT"
else
    echo "❌ ERROR: 没有在 build/ 下找到可执行文件 '$TARGET_NAME'"
    exit 1
fi

echo "=============================="
echo "  RUNNING EXECUTABLE..."
echo "=============================="
cd "$PROJECT_ROOT"
./"$TARGET_NAME"

echo "=============================="
echo "  ✅ ALL DONE."
echo "=============================="
