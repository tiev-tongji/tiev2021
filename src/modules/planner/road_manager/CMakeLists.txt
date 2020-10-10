find_package(libpqxx REQUIRED)
set(road_manager_src ${road_manager_src} road_manager.cpp)
add_library(road_manager ${road_manager_src})

add_executable(road_manager_test road_manager_test.cpp)
target_link_libraries(road_manager nature planner_tools libpqxx.so)
target_link_libraries(road_manager_test road_manager message_manager ${OpenCV_LIBS})
