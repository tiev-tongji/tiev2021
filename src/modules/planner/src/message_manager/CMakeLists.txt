add_library(message_manager SHARED "")

target_sources(message_manager PRIVATE 
    message_manager.cpp 
)

target_include_directories(message_manager PUBLIC 
    ./
    ${CMAKE_SOURCE_DIR}/../../msg
)

target_link_libraries(message_manager PUBLIC
    visualization_zcm
    planner_common
    speed_planner
)