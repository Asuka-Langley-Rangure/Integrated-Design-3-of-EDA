cmake_minimum_required(VERSION 3.14)

# 工程名随意
project(placementDesigner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -------------------------------
# 可执行文件（主程序）
# -------------------------------
add_executable(
    placementDesigner
    ${PROJECT_SOURCE_DIR}/main.cpp
    ${PROJECT_SOURCE_DIR}/parse_bookshelf.cpp

    # 转换器
    ${PROJECT_SOURCE_DIR}/DefPlaceAdapter/DefPlaceAdapter.cpp

    # 核心放置实现
    ${PROJECT_SOURCE_DIR}/MyPlace/myplace.cpp

    # 算法模块
    ${PROJECT_SOURCE_DIR}/algorithm/wirelength.cpp
    ${PROJECT_SOURCE_DIR}/algorithm/density.cpp

    # 解析器
    ${PROJECT_SOURCE_DIR}/parser/defParser.cpp

    # 绘图模块（和示例一样，直接编进可执行文件）
    ${PROJECT_SOURCE_DIR}/Plot/plot.cpp
)

# -------------------------------
# 头文件路径（参考你给的示例写法）
# -------------------------------
target_include_directories(placementDesigner PRIVATE
    ${PROJECT_SOURCE_DIR}/MyPlace          # myplace.h
    ${PROJECT_SOURCE_DIR}/PlaceData        # placedb.h
    ${PROJECT_SOURCE_DIR}/Common           # common.h, objects.h
    ${PROJECT_SOURCE_DIR}/algorithm        # db.h, wirelength.h, density.h

    ${PROJECT_SOURCE_DIR}/DefPlaceAdapter  # DefPlaceAdapter.h
    
    ${PROJECT_SOURCE_DIR}/parser           # defParser.h
    ${PROJECT_SOURCE_DIR}/eigen3           # <Eigen/...>
    ${PROJECT_SOURCE_DIR}/Plot             # plot.h
    ${PROJECT_SOURCE_DIR}/Library/CImg     # "CImg.h"
)
