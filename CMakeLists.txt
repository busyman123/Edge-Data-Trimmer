cmake_minimum_required(VERSION 3.20)
project(NativeEdgeDataTrimmer VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Arrow REQUIRED)
find_package(Parquet REQUIRED)

add_executable(edge_trimmer
    src/main.cpp
    src/mcap_writer.cpp
    src/parquet_cursor.cpp
    src/socket_uplink.cpp
    src/smart_trimmer.cpp
    src/waymo_pipeline.cpp
)

target_include_directories(edge_trimmer PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(edge_trimmer PRIVATE
    Arrow::arrow_shared
    Parquet::parquet_shared
)
