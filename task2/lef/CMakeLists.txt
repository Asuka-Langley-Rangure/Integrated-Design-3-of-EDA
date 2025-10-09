cmake_minimum_required(VERSION 3.10)
project(lef_example)

set(CMAKE_CXX_STANDARD 11)

add_executable(lef_example
    main.cpp
    LefFile.cpp
    LefParser.cpp
    LefFile.h
    LefParser.h
)