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

# Install
install(TARGETS xray_to_ozz_converter
    RUNTIME DESTINATION bin
)