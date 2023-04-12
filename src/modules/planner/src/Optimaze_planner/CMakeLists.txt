cmake_minimum_required(VERSION 3.0)
find_package(OsqpEigen REQUIRED)
add_executable(example example.cpp)
target_link_libraries(example OsqpEigen::OsqpEigen)


find_package(OpenCV REQUIRED)
target_link_libraries(example ${OpenCV_LIBS})