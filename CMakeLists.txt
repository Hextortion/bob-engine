cmake_minimum_required(VERSION 3.15)

project(bob LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_executable(bob main.cpp engine.cpp)
