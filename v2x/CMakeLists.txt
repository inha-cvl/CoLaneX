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

set( HLVSRC ./hlv.c)
set( TLVSRC ./tlv.c)

add_executable(
    hlv ${HLVSRC}
)
add_executable(
    tlv ${TLVSRC}
)

set_source_files_properties( ${HLVSRC} PROPERTIES LANGUAGE CXX )
set_source_files_properties( ${TLVSRC} PROPERTIES LANGUAGE CXX )


target_link_libraries(hlv ${catkin_LIBRARIES} asncodec)
target_link_libraries(tlv ${catkin_LIBRARIES} asncodec)