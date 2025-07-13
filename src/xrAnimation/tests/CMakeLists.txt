cmake_minimum_required(VERSION 3.8)

project(xrAnimationTests)

# Simple test for animation playback fix
add_executable(test_animation_playback_simple
    test_animation_playback_simple.cpp
)

target_link_libraries(test_animation_playback_simple
    PRIVATE
    xrAnimation
    xrCore
    xrMiscMath  # For Fquaternion::set
    ozz_animation
    ozz_base
)

target_include_directories(test_animation_playback_simple
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/include
)

# Copy test data
file(COPY test_data DESTINATION ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

# Bind pose comparison test
add_executable(test_bind_pose_comparison test_bind_pose_comparison.cpp)
target_link_libraries(test_bind_pose_comparison xrAnimation xrCore xrMiscMath ozz_animation ozz_base)

# Skeleton viewer test
add_executable(test_skeleton_viewer test_skeleton_viewer.cpp)
target_link_libraries(test_skeleton_viewer ozz_animation ozz_base)





# Set properties
set_target_properties(test_animation_playback_simple test_bind_pose_comparison test_skeleton_viewer PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}
)