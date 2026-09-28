cmake_minimum_required(VERSION 3.16)
project(DiffDrivePID LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Prefer modern GLVND OpenGL libs on Linux
set(OpenGL_GL_PREFERENCE GLVND)

find_package(OpenGL REQUIRED)
find_package(Threads REQUIRED)

# Use system GLFW if available, otherwise download and build it
find_package(glfw3 3.3 QUIET)
if(NOT glfw3_FOUND)
    message(STATUS "System GLFW not found - fetching from GitHub")
    include(FetchContent)
    set(GLFW_BUILD_DOCS     OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_TESTS    OFF CACHE BOOL "" FORCE)
    set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    FetchContent_Declare(glfw
        GIT_REPOSITORY https://github.com/glfw/glfw.git
        GIT_TAG        3.4)
    FetchContent_MakeAvailable(glfw)
endif()

# Console-only version (no graphics)
add_executable(robot diff_drive_pid.cpp)

# OpenGL visualization
add_executable(robot_viz diff_drive_pid_viz.cpp)
target_link_libraries(robot_viz PRIVATE glfw OpenGL::GL Threads::Threads)

foreach(t robot robot_viz)
    if(MSVC)
        target_compile_options(${t} PRIVATE /W4)
    else()
        target_compile_options(${t} PRIVATE -Wall -Wextra)
    endif()
endforeach()