# MIAO Graph Optimization Library
# Adapted from lightning-lm for FAST-LIVO2

cmake_minimum_required(VERSION 3.10)

# Library sources
set(MIAO_SOURCES
    core/graph/graph.cc
    core/graph/optimizer.cc
    core/common/string_tools.cc
    core/math/marginal_covariance_cholesky.cc
    core/opti_algo/optimization_algorithm.cc
    core/opti_algo/opti_algo_gauss_newton.cc
    core/opti_algo/opti_algo_lm.cc
    core/opti_algo/opti_algo_dogleg.cc
    core/solver/solver.cc
)

# Create the library
add_library(miao STATIC ${MIAO_SOURCES})

# Include directories - put MIAO's own paths FIRST to avoid conflicts with project's include/common
target_include_directories(miao PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
)
# Private include to ensure MIAO's common/ is searched before project's include/common/
target_include_directories(miao PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Link dependencies
target_link_libraries(miao PUBLIC
    ${GLOG_LIBRARIES}
    TBB::tbb
    Sophus::Sophus
)

# Set C++ standard
target_compile_features(miao PUBLIC cxx_std_17)

# Suppress warnings from MIAO library
target_compile_options(miao PRIVATE -Wno-unused-parameter -Wno-reorder)
