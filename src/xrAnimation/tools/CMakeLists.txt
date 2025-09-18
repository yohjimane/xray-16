cmake_minimum_required(VERSION 3.8)

project(xrAnimationTools)

# X-Ray to ozz converter tool
add_executable(xray_to_ozz_converter
    xray_to_ozz_converter.cpp
)

target_link_libraries(xray_to_ozz_converter
    PRIVATE
    xrAnimation
    xrCore
    xrEngine
    xrScriptEngine
    ozz_animation_offline
    ozz_animation
    ozz_base
)

target_include_directories(xray_to_ozz_converter
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/imgui
)

# Set properties
set_target_properties(xray_to_ozz_converter PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}
)

# ozz animation viewer tool (using ozz playback sample)
add_executable(ozz_animation_viewer
    ozz_animation_viewer.cpp
)

# Link to the existing ozz sample_framework
target_link_libraries(ozz_animation_viewer
    PRIVATE
    sample_framework  # From ozz-animation
    xrAnimation
    xrCore
    ozz_animation
    ozz_base
)

target_include_directories(ozz_animation_viewer
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/samples
)

# Set properties
set_target_properties(ozz_animation_viewer PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}
)

# Install
install(TARGETS xray_to_ozz_converter ozz_animation_viewer
    RUNTIME DESTINATION bin
)
