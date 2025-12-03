cmake_minimum_required(VERSION 3.10)
project(MossStabilityEngine C)

set(CMAKE_C_STANDARD 99)

include_directories(include)

# Library
add_library(moss_stability STATIC src/reason_solver.c src/ternary_core.c)

# Example Executable
add_executable(flight_sim examples/flight_loop_sim.c)
target_link_libraries(flight_sim moss_stability m)
