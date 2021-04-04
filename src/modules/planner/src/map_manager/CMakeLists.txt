add_library(map_manager SHARED "")

target_sources(map_manager PRIVATE 
    map_manager.cpp
)

target_include_directories(map_manager PRIVATE 
    ${CMAKE_SOURCE_DIR}/include/map_manager
    ${CMAKE_SOURCE_DIR}/include/routing
    ${CMAKE_SOURCE_DIR}/include/planner_common
    ${CMAKE_SOURCE_DIR}/include/message_manager
    ${CMAKE_SOURCE_DIR}/include/speed_planner
    ${CMAKE_SOURCE_DIR}/../../msg
)

target_link_libraries(map_manager PUBLIC 
    message_manager
)