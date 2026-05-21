cmake_minimum_required(VERSION 3.21)

project(CorBiCore LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(simpleble CONFIG QUIET)
if(NOT simpleble_FOUND)
    include(FetchContent)
    FetchContent_Declare(
        simpleble
        GIT_REPOSITORY https://github.com/OpenBluetoothToolbox/SimpleBLE.git
        GIT_TAG v0.6.1
        SOURCE_SUBDIR simpleble
    )
    FetchContent_MakeAvailable(simpleble)
endif()

set(
    main.cpp
)

add_executable(CorBiCore main.cpp)

target_link_libraries(CorBiCore simpleble::simpleble)

if(MSVC)
    # Force to always compile with W4
    if(CMAKE_CXX_FLAGS MATCHES "/W[0-4]")
        string(REGEX REPLACE "/W[0-4]" "/W4" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
    else()
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4")
    endif()
elseif(CMAKE_COMPILER_IS_GNUCC OR CMAKE_COMPILER_IS_GNUCXX)
    # Update if necessary
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-long-long -pedantic")
endif()
