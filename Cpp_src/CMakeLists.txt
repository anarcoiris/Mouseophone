cmake_minimum_required(VERSION 3.15)
project(mouse_mic CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

# vcpkg: cmake -DCMAKE_TOOLCHAIN_FILE=<vcpkg_root>/scripts/buildsystems/vcpkg.cmake ..
find_package(portaudio CONFIG REQUIRED)

add_executable(mouse_mic mouse_mic.cpp)

target_link_libraries(mouse_mic PRIVATE
    portaudio
    user32
)

# Math constant M_PI on MSVC
target_compile_definitions(mouse_mic PRIVATE _USE_MATH_DEFINES)

# Optimization
if(MSVC)
    target_compile_options(mouse_mic PRIVATE /O2 /W3)
endif()
