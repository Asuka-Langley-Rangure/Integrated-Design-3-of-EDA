cmake_minimum_required(VERSION 3.14)

project(myplacement)

add_executable(
    myplace
    myplace_main.cpp
    parse_bookshelf.cpp
    ${PROJECT_SOURCE_DIR}/MyPlace/myplace.cpp
    ${PROJECT_SOURCE_DIR}/Plot/plot.cpp
)

# 3) 头文件路径（注意：eigen3 指到“包含 Eigen/ 子目录的那层”）
target_include_directories(myplace PRIVATE
    ${PROJECT_SOURCE_DIR}/MyPlace
    ${PROJECT_SOURCE_DIR}/PlaceData
    ${PROJECT_SOURCE_DIR}/Common
    ${PROJECT_SOURCE_DIR}/eigen3         # 用 <Eigen/...>
    ${PROJECT_SOURCE_DIR}/Plot
    ${PROJECT_SOURCE_DIR}/Library/CImg    # 用 "CImg.h"
)
