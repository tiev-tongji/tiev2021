set(TimeLock_src ${TimeLock_src} time_lock.cpp)
add_library(time_lock ${TimeLock_src})

add_executable(time_lock_test time_lock_test.cpp)
target_link_libraries(time_lock_test time_lock)
