cmake_minimum_required(VERSION 3.0.2)
project(v2x LANGUAGES CXX)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs geometry_msgs genmsg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
    INCLUDE_DIRS include include/j2735
    LIBRARIES include/j2735
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    include/j2735
)

link_directories(
    include/j2735
)

set(
    SOURCES ./hlv.c
)
add_executable(
    v2x ${SOURCES}
)

set_source_files_properties( ${SOURCES} PROPERTIES LANGUAGE CXX )

target_link_libraries(
    v2x ${catkin_LIBRARIES} asncodec
)