find_package(gRPC CONFIG REQUIRED)
add_library(routing STATIC "")
target_sources(routing PRIVATE
    Routing.cpp
)
target_include_directories(routing PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/routing
)
target_link_libraries(routing PUBLIC 
    planner_common routing_service gRPC::grpc++
)
