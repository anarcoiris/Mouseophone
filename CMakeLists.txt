cmake_minimum_required(VERSION 3.16)
project(Mouseofono CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Core Sources
set(CORE_SOURCES
    src/core/signal_reconstructor.cpp
    src/core/processor_chain.cpp
    src/core/wav_writer.cpp
)

# Platform specific
if(WIN32)
    set(PLATFORM_SOURCES
        src/platforms/windows/raw_input_reader.cpp
    )
    set(PLATFORM_LIBS user32)
else()
    # Placeholder for Linux
    set(PLATFORM_SOURCES
        # src/platforms/linux/evdev_reader.cpp
    )
endif()

add_executable(mouseofono
    src/main.cpp
    ${CORE_SOURCES}
    ${PLATFORM_SOURCES}
)

target_include_directories(mouseofono PRIVATE src)
target_link_libraries(mouseofono PRIVATE ${PLATFORM_LIBS})

if(MSVC)
    target_compile_definitions(mouseofono PRIVATE _USE_MATH_DEFINES)
    target_compile_options(mouseofono PRIVATE /O2 /W3)
endif()
