add_library(planner_context STATIC "")
target_sources(planner_context PRIVATE
    decision_context.cpp
)
target_include_directories(planner_context PUBLIC 
    ./
)
target_link_libraries(planner_context PUBLIC 
    planner_common message_manager
)
