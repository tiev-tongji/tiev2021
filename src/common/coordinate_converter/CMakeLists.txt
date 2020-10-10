set(coordinate_converter_src ${coordinate_converter_src} basic_coordinate_converter.cpp)
set(coordinate_converter_src ${coordinate_converter_src} coordinate_converter.cpp)
add_library(coordinate_converter ${coordinate_converter_src})

#add_executable(angle_test angle_test.cpp)
#target_link_libraries(angle_test nature)
