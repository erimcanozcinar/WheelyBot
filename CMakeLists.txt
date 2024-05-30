cmake_minimum_required(VERSION 3.10)
project(wheelyBot_Sim LANGUAGES CXX)
set(CMAKE_PREFIX_PATH "/home/erim/raisim_ws_v1.1.7/install")

find_package(raisim CONFIG REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} "/usr/local/include")
link_directories("/usr/local/lib")

add_executable(wheelyBot src/main.cpp 
                         src/simulation.cpp
                         src/robot.cpp 
                         src/control.cpp 
                         src/math.cpp)

target_link_libraries(wheelyBot PUBLIC raisim::raisim pthread)
target_include_directories(wheelyBot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)