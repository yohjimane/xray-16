cmake_minimum_required(VERSION 3.8)

enable_testing()

set(OZZ_ROOT ${CMAKE_SOURCE_DIR}/Externals/ozz-animation)
set(OZZ_SAMPLES_ROOT ${OZZ_ROOT}/samples)

if (NOT TARGET gtest)
    add_subdirectory(${OZZ_ROOT}/extern/gtest ${CMAKE_BINARY_DIR}/ozz_gtest)
endif()

add_executable(xrAnimation_converter_tests
    test_converter.cpp
    ozz_kinematics_test_main.cpp)

target_include_directories(xrAnimation_converter_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${OZZ_ROOT}/include
    ${OZZ_SAMPLES_ROOT}
    ${CMAKE_SOURCE_DIR}/Externals/tracy/public)

target_link_libraries(xrAnimation_converter_tests PRIVATE
    xrAnimation
    ozz_animation
    ozz_animation_offline
    ozz_base
    sample_framework
    xrEngine
    xrScriptEngine
    gtest)

target_compile_features(xrAnimation_converter_tests PRIVATE cxx_std_17)

target_compile_definitions(xrAnimation_converter_tests PRIVATE
    PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

set_target_properties(xrAnimation_converter_tests PROPERTIES
    FOLDER "xrAnimation/Tests")

add_test(NAME xrAnimation_converter_tests COMMAND xrAnimation_converter_tests)

add_executable(ozz_kinematics_tests
    ozz_kinematics_parity_tests.cpp
    ozz_kinematics_test_main.cpp)

target_include_directories(ozz_kinematics_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_SOURCE_DIR}/src
    ${OZZ_ROOT}/include
    ${CMAKE_SOURCE_DIR}/Externals/tracy/public)

target_link_libraries(ozz_kinematics_tests PRIVATE
    xrAnimation
    ozz_animation
    ozz_base
    gtest
    xrEngine
    xrScriptEngine)

target_compile_features(ozz_kinematics_tests PRIVATE cxx_std_17)

target_compile_definitions(ozz_kinematics_tests PRIVATE
    PROJECT_ROOT="${CMAKE_SOURCE_DIR}")

set_target_properties(ozz_kinematics_tests PROPERTIES
    FOLDER "xrAnimation/Tests")

add_test(NAME ozz_kinematics_tests COMMAND ozz_kinematics_tests)
