cmake_minimum_required(VERSION 3.8)

add_executable(xrAnimation_converter_tests
    test_converter.cpp)

# Resolve ozz samples include directory (submodule lives alongside xray-16).
get_filename_component(OZZ_SAMPLES_ROOT ${CMAKE_SOURCE_DIR}/../ozz-animation/samples ABSOLUTE)

# Provide include paths for dependency headers.
target_include_directories(xrAnimation_converter_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/Externals/ozz-animation/include
    ${OZZ_SAMPLES_ROOT})

target_link_libraries(xrAnimation_converter_tests PRIVATE
    xrAnimation
    ozz_animation
    ozz_animation_offline
    ozz_base
    sample_framework
    xrEngine
    xrScriptEngine)

target_compile_features(xrAnimation_converter_tests PRIVATE cxx_std_17)

target_compile_definitions(xrAnimation_converter_tests PRIVATE
    PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

set_target_properties(xrAnimation_converter_tests PROPERTIES
    FOLDER "xrAnimation/Tests")
