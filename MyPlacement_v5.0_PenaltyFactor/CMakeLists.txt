# cmake needs this line
cmake_minimum_required(VERSION 3.5)

# Define project name
project(myplacement)

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
