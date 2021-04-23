add_library(map_manager STATIC "")

target_sources(map_manager PRIVATE 
    map_manager.cpp
)

target_include_directories(map_manager PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/map_manager
)

target_link_libraries(map_manager PUBLIC 
    message_manager
    routing
)