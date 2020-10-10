set(time_counter_src ${time_counter_src} time_counter.cpp)
add_library(time_counter ${time_counter_src})

add_executable(time_counter_test time_counter_test.cpp)
target_link_libraries(time_counter_test time_counter time_lock)
