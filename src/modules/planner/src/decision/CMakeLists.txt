aux_source_directory(. SRC_DIR)
aux_source_directory(./fsm_states SRC_DIR)

add_library(decision STATIC "")
target_sources(decision PRIVATE 
    ${SRC_DIR}
)
target_include_directories(decision PRIVATE 
    ${CMAKE_SOURCE_DIR}/include/decision
)
target_include_directories(decision PRIVATE
    map_manager
)