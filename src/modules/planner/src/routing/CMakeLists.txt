find_package(gRPC REQUIRED)
add_library(routing STATIC "")
target_sources(routing PRIVATE
    Routing.cpp
)
target_include_directories(routing PUBLIC 
    ./
)
target_link_libraries(routing PUBLIC 
    planner_common routing_service gRPC::grpc++
)
