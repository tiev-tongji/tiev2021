add_library(map_manager STATIC "")
find_package(OpenCV REQUIRED)

target_sources(map_manager PRIVATE 
    map_manager.cpp
)

target_include_directories(map_manager PUBLIC 
    ./
)

target_link_libraries(map_manager PUBLIC 
    message_manager
    routing
)