cmake_minimum_required(VERSION 3.10)
project(placementDesigner)
set(CMAKE_CXX_STANDARD 17)

# -------------------------------
# 可执行文件（主程序）
# -------------------------------
add_executable(placementDesigner
    main.cpp

    # 核心算法模块
    algorithm/wirelength.cpp
    algorithm/density.cpp
    algorithm/db.cpp

    # 解析器
    parser/defParser.cpp
)

# -------------------------------
# 绘图模块（单独编译）
# -------------------------------
# add_library(plot STATIC Plot/plot.cpp)

# plot 只需要 db.h 和 CImg，不包含 Eigen
# target_include_directories(plot PRIVATE
#     ${PROJECT_SOURCE_DIR}/placement
#     ${PROJECT_SOURCE_DIR}/Library/CImg
# )

add_library(plot STATIC Plot/plot.cpp)

# plot 只需要 db.h 和 CImg，不包含 Eigen
target_include_directories(plot PRIVATE
    ${PROJECT_SOURCE_DIR}/placement
    ${PROJECT_SOURCE_DIR}/Library/CImg
)

# -------------------------------
# 主程序包含路径
# -------------------------------
target_include_directories(placementDesigner PRIVATE
    ${CMAKE_SOURCE_DIR}/core
    ${CMAKE_SOURCE_DIR}/parser
    ${CMAKE_SOURCE_DIR}/algorithm
    ${PROJECT_SOURCE_DIR}/Common
    ${PROJECT_SOURCE_DIR}/placement
)

# -------------------------------
# 链接库
# -------------------------------
target_link_libraries(placementDesigner PRIVATE plot)
