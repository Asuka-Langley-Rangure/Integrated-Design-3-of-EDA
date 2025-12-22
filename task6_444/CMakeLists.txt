cmake_minimum_required(VERSION 3.14)

project(myplacement LANGUAGES CXX)

# C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 输出 compile_commands.json，方便 VSCode
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# 可执行文件
add_executable(myplace
    myplace_main.cpp
    parse_bookshelf.cpp

    MyPlace/myplace.cpp
    PlaceData/placedata.cpp
    Plot/plot.cpp

    FFT/fft.cpp
    FFT/fftsg.cpp
    FFT/fftsg2d.cpp

    Common/object.cpp

    Optimization/NesterovOpt.cpp
)

# 头文件路径
target_include_directories(myplace PRIVATE
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/MyPlace
    ${PROJECT_SOURCE_DIR}/PlaceData
    ${PROJECT_SOURCE_DIR}/Plot
    ${PROJECT_SOURCE_DIR}/Common
    ${PROJECT_SOURCE_DIR}/FFT
    ${PROJECT_SOURCE_DIR}/Optimization

    # Eigen: include 目录应当是包含 "Eigen/" 子目录的那一层
    ${PROJECT_SOURCE_DIR}/eigen3

    # CImg.h 所在目录
    ${PROJECT_SOURCE_DIR}/Library/CImg
)

# Linux 链接库
if(UNIX AND NOT APPLE)
    target_link_libraries(myplace PRIVATE pthread m dl X11)
endif()
