cmake_minimum_required(VERSION 3.16)

project(test_skeleton_viewer_standalone LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add only ozz-animation to avoid pulling full engine dependencies
set(OZZ_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../../Externals/ozz-animation")
if (NOT EXISTS "${OZZ_DIR}/CMakeLists.txt")
    message(FATAL_ERROR "ozz-animation not found at: ${OZZ_DIR}")
endif()

set(ozz_build_tools OFF CACHE BOOL "" FORCE)
set(ozz_build_fbx OFF CACHE BOOL "" FORCE)
set(ozz_build_gltf OFF CACHE BOOL "" FORCE)
set(ozz_build_data OFF CACHE BOOL "" FORCE)
set(ozz_build_samples OFF CACHE BOOL "" FORCE)
set(ozz_build_howtos OFF CACHE BOOL "" FORCE)
set(ozz_build_tests OFF CACHE BOOL "" FORCE)
set(ozz_build_simd_ref OFF CACHE BOOL "" FORCE)
add_subdirectory("${OZZ_DIR}" "${CMAKE_BINARY_DIR}/ozz-animation")

add_executable(test_skeleton_viewer
    ../test_skeleton_viewer.cpp
)

target_include_directories(test_skeleton_viewer PRIVATE
    "${OZZ_DIR}/include"
)

target_link_libraries(test_skeleton_viewer PRIVATE
    ozz_animation
    ozz_base
)

set_target_properties(test_skeleton_viewer PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)
