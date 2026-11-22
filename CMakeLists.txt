cmake_minimum_required(VERSION 3.20)
project(waymo_edge_trimmer LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)

add_executable(waymo_edge_trimmer src/main.cpp)
target_link_libraries(waymo_edge_trimmer PRIVATE Arrow::arrow_shared Parquet::parquet_shared)
