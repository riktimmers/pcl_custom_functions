project(pcl_custom_functions)
cmake_minimum_required(VERSION 3.0.2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS -Wall)

find_package(PCL 1.10 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)
  
set(source 
"src/main.cpp"
"src/pcl_test.cpp"
"src/pcl_custom_test.cpp"
)

add_executable(main ${source})
target_link_libraries(main ${PCL_LIBRARIES})