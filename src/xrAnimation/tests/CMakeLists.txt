cmake_minimum_required(VERSION 3.8)

project(xrAnimationTests)

# Test executable for ozz integration
add_executable(test_ozz_integration
    test_ozz_integration.cpp
)

target_link_libraries(test_ozz_integration
    PRIVATE
    xrAnimation
    xrCore
    xrEngine
    ozz_animation_offline
    ozz_animation
    ozz_base
)

target_include_directories(test_ozz_integration
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/imgui
)

# Test with basic ozz functionality
add_executable(test_ozz_basic
    test_ozz_basic.cpp
)

target_link_libraries(test_ozz_basic
    PRIVATE
    xrAnimation
    xrCore
    ozz_animation
    ozz_base
)

target_include_directories(test_ozz_basic
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/include
)

# Test with real X-Ray data
add_executable(test_real_data_conversion
    test_real_data_conversion.cpp
)

target_link_libraries(test_real_data_conversion
    PRIVATE
    xrAnimation
    xrCore
    xrEngine
    ozz_animation_offline
    ozz_animation
    ozz_base
)

target_include_directories(test_real_data_conversion
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../..
    ${CMAKE_CURRENT_SOURCE_DIR}/../../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/ozz-animation/include
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../Externals/imgui
)

# Copy test data
file(COPY test_data DESTINATION ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE})

# Set properties
set_target_properties(test_ozz_integration test_ozz_basic test_real_data_conversion PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${CMAKE_BUILD_TYPE}
)