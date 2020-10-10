set(message_manager_src ${message_manager_src} message_manager.cpp)
add_library(message_manager ${message_manager_src})

add_executable(message_manager_test message_manager_test.cpp)
target_link_libraries(message_manager planner_tools zcm nature coordinate_converter)
target_link_libraries(message_manager_test message_manager)
