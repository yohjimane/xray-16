cmake_minimum_required(VERSION 3.8)

project(xrAnimation)

# Set up ozz-animation dependency
find_package(ozz REQUIRED)

# Include directories
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../Include
    ${CMAKE_CURRENT_SOURCE_DIR}/../xrCore
    ${CMAKE_CURRENT_SOURCE_DIR}/../xrEngine
    ${CMAKE_CURRENT_SOURCE_DIR}/../Common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../ozz-animation/include
)

# Source files
set(SOURCES
    AnimationConverter.cpp
    AnimationConverter.h
    OGFConverter.cpp
    OGFConverter.h
    OzzAnimationSystem.cpp
    OzzAnimationSystem.h
    OzzKinematicsAnimated.cpp
    OzzKinematicsAnimated.h
    stdafx.cpp
    stdafx.h
)

# Create static library
add_library(xrAnimation STATIC ${SOURCES})

# Link libraries
target_link_libraries(xrAnimation
    PRIVATE
    xrCore
    ozz::animation_offline
    ozz::animation_runtime
    ozz::base
)

# Set target properties
set_target_properties(xrAnimation PROPERTIES
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
    FOLDER "xrAnimation"
)

# Compiler-specific options
if(MSVC)
    target_compile_options(xrAnimation PRIVATE
        /MP  # Multi-processor compilation
        /W3  # Warning level 3
        /wd4996  # Disable deprecated function warnings
    )
    
    # Use precompiled headers
    target_precompile_headers(xrAnimation PRIVATE stdafx.h)
endif()

# Install
install(TARGETS xrAnimation
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)