cmake_minimum_required(VERSION 3.8)

add_executable(xrAnimation_converter_tests
    test_converter.cpp)

# Provide include paths for dependency headers.
target_include_directories(xrAnimation_converter_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/Externals/ozz-animation/include)

target_link_libraries(xrAnimation_converter_tests PRIVATE
    ozz_animation
    ozz_base)

target_compile_features(xrAnimation_converter_tests PRIVATE cxx_std_17)

get_filename_component(XRAY_WORKSPACE_ROOT ${CMAKE_SOURCE_DIR} DIRECTORY)

target_compile_definitions(xrAnimation_converter_tests PRIVATE
    WORKSPACE_ROOT="${XRAY_WORKSPACE_ROOT}")

set_target_properties(xrAnimation_converter_tests PROPERTIES
    FOLDER "xrAnimation/Tests")
