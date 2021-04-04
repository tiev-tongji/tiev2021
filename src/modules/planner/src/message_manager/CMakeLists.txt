add_library(message_manager SHARED "")

target_sources(message_manager PRIVATE 
    message_manager.cpp 
)

target_include_directories(message_manager PRIVATE 
    ${CMAKE_SOURCE_DIR}/include/message_manager
    ${CMAKE_SOURCE_DIR}/include/planning_common
    ${CMAKE_SOURCE_DIR}/include/visualization_msg
    ${CMAKE_SOURCE_DIR}/include/speed_planner
    ../../${CMAKE_SOURCE_DIR}/msg
)