add_library(routing SHARED "")
target_sources(routing PRIVATE
    Routing.cpp
)
target_include_directories(routing PUBLIC 
    ${CMAKE_SOURCE_DIR}/include/routing
)
target_link_libraries(routing PUBLIC 
    pqxx 
    pq
)
