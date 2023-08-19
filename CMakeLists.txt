cmake_minimum_required(VERSION 3.24)
project(Blossom)

set(CMAKE_CXX_STANDARD 20)

add_executable(Blossom main.cpp Graph.h Graph.cpp blossom.h)
