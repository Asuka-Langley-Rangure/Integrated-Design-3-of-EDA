# cmake needs this line
cmake_minimum_required(VERSION 3.5)

# Define project name
project(myplacement)

# Declare the executable target built from your sources
add_executable(
    myplace
    myplace_main.cpp
    ${PROJECT_SOURCE_DIR}/MyPlace/myplace.cpp
    #${PROJECT_SOURCE_DIR}/PlaceData/placedata.cpp
)
include_directories (
    ${PROJECT_SOURCE_DIR}/MyPlace
    ${PROJECT_SOURCE_DIR}/PlaceData
    ${PROJECT_SOURCE_DIR}/Common
    ${PROJECT_SOURCE_DIR}/egien3
    )

# Link to libraries if needed
