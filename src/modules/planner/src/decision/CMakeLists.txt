aux_source_directory(. SRC_DIR)
aux_source_directory(./fsm_states SRC_DIR)
add_library(decision ${SRC_DIR})