project(Path_Planning)

cmake_minimum_required (VERSION 3.5)
include("cmake/FindGlog.cmake")
include("cmake/FindGflags.cmake")

add_definitions(-std=c++11)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

set(sources src/main.cpp
            src/planner/src/curveSmoother.cpp
            src/planner/src/end_condition_sampler.cpp
            src/planner/src/feasible_region.cpp
            src/planner/src/feasible_region.cpp
            src/planner/src/JMT_curve1d.cpp
            src/planner/src/JMT_gflags.cpp
            src/planner/src/JMT_planner.cpp
            src/planner/src/quartic_polynomial_curve1d.cpp
            src/planner/src/quintic_polynomial_curve1d.cpp
            src/planner/src/trajectory_decouple_generator.cpp
            src/planner/src/trajectory_evaluator.cpp
            src/planner/src/TopologyManager.cpp
            src/common/math/cartesian_frenet_conversion.cpp
            src/common/math/math_utils.cpp
            src/common/struct/traj_structs.cpp
        )

include_directories(/usr/local/include)
include_directories(src)
include_directories(src/planner/include)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1.11.0/lib)

endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 


add_executable(path_planning ${sources})

target_link_libraries(path_planning z ssl uv uWS)
