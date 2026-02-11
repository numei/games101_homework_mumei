cmake_minimum_required(VERSION 4.1)
project (00)

find_package(Eigen3 REQUIRED)
include_directories("/opt/homebrew/include")

add_executable (main main.cpp)
