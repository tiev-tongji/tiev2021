aux_source_directory(. SRC_DIR)
aux_source_directory(./fsm_states SRC_DIR)

add_library(decision SHARED "")
target_sources(decision PRIVATE 
    ${SRC_DIR}
)
target_include_directories(decision PRIVATE 
    ${CMAKE_SOURCE_DIR}/include/decision
)