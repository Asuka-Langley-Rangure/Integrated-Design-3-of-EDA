# cmake needs this line
cmake_minimum_required(VERSION 3.5)

# Define project name
project(myplacement LANGUAGES CXX)

# 建议：导出 compile_commands.json 便于 IDE/静态分析
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# C++ 标准与常用编译选项
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
if(MSVC)
  add_compile_options(/W4)
else()
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 源码与目标
add_executable(myplace
    ${PROJECT_SOURCE_DIR}/main.cpp
    ${PROJECT_SOURCE_DIR}/parse_bookshelf.cpp
    # 如后续新增实现文件可在此追加
    # ${PROJECT_SOURCE_DIR}/PlaceData/placedata.cpp
    # ${PROJECT_SOURCE_DIR}/MyPlace/myplace.cpp
)

# 头文件包含路径（和你的目录结构一致）
target_include_directories(myplace PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/Common
    ${PROJECT_SOURCE_DIR}/MyPlace
    ${PROJECT_SOURCE_DIR}/PlaceData
)

# Linux 平台常用库（按你原来的写法）
if(UNIX AND NOT APPLE)
  target_link_libraries(myplace PRIVATE m dl X11)
endif()

# 兼容旧版 libstdc++ 的 <filesystem>（GCC < 9 需要显式链接）
if (CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  if (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(myplace PRIVATE stdc++fs)
  endif()
endif()

# 将可执行文件输出到项目根目录（等价于你之前 shell 脚本“剪切回根目录”）
# set_target_properties(myplace PROPERTIES
#   RUNTIME_OUTPUT_DIRECTORY "${PROJECT_SOURCE_DIR}"
# )

# 可选：安装规则（需要时用 `cmake --install`）
# install(TARGETS myplace RUNTIME DESTINATION .)
