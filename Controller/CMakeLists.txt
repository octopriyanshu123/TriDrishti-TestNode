cmake_minimum_required(VERSION 3.10)
project(diff_drive_line_follower LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

# Prefer the modern GLVND OpenGL libraries on Linux
set(OpenGL_GL_PREFERENCE GLVND)

find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)     # freeglut: sudo apt install freeglut3-dev

add_executable(diff_drive_line_follower src/main.cpp)

target_link_libraries(diff_drive_line_follower
  PRIVATE
    OpenGL::GL
    OpenGL::GLU
    GLUT::GLUT
    m
)

target_compile_options(diff_drive_line_follower PRIVATE -Wall -Wextra)