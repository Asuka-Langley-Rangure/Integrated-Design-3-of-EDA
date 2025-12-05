# cmake needs this line
cmake_minimum_required(VERSION 3.5)


# 设置 build type 为 Debug（单配置 generator 下有效）
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Build type" FORCE)
endif()

# 只为 Debug 构建指定额外 flags（不用直接覆盖 CMAKE_CXX_FLAGS）
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_C_FLAGS_DEBUG   "-g -O0")  # 如果你也有 C 文件

# （可选）对所有构建都加一些通用 flags，比如 warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

# Declare the executable target built from your sources
add_executable(
    myplace
    myplace_main.cpp
    ${PROJECT_SOURCE_DIR}/MyPlace/myplace.cpp
    ${PROJECT_SOURCE_DIR}/PlaceData/placedata.cpp
    ${PROJECT_SOURCE_DIR}/Plot/plot.cpp
    ${PROJECT_SOURCE_DIR}/FFT/fft.cpp
    ${PROJECT_SOURCE_DIR}/FFT/fftsg.cpp
    ${PROJECT_SOURCE_DIR}/FFT/fftsg2d.cpp
    ${PROJECT_SOURCE_DIR}/Common/object.cpp
    ${PROJECT_SOURCE_DIR}/Optimization/NesterovOpt.cpp
)
include_directories (
    ${PROJECT_SOURCE_DIR}/MyPlace
    ${PROJECT_SOURCE_DIR}/PlaceData
    ${PROJECT_SOURCE_DIR}/Plot
    ${PROJECT_SOURCE_DIR}/Common
    ${PROJECT_SOURCE_DIR}/Library/CImg
    ${PROJECT_SOURCE_DIR}/FFT
    ${PROJECT_SOURCE_DIR}/Optimization
    )

target_link_libraries(myplace "-lpthread -lm -ldl -lX11")

