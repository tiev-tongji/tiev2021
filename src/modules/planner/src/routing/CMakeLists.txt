find_package(gRPC CONFIG REQUIRED)
add_library(routing STATIC "" rest_client.cpp)
target_sources(routing PRIVATE
    Routing.cpp
)
target_include_directories(routing PUBLIC 
    ./
)
target_link_libraries(routing PUBLIC 
    planner_common routing_service gRPC::grpc++
)
