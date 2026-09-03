cmake_minimum_required(VERSION 3.10)
project(DogLegs)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_RULE_MESSAGES OFF)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DOGLEGS_OUTPUT_TO_SOURCE_DIR "Place build artifacts in the source tree bin/ and lib/ directories" OFF)

if(DOGLEGS_OUTPUT_TO_SOURCE_DIR)
    set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
    set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
endif()

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
    add_definitions( -D_USE_MATH_DEFINES=True)
    add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
    add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
    set(CMAKE_CXX_FLAGS "-Wno-unused-result")
endif()

file(GLOB_RECURSE SOURCE
        src/main.cpp
        src/fileio/fileloader.cpp
        src/fileio/filesaver.cpp
        src/fileio/rosbagloader.cpp
        src/imufusion/inspropagation.cpp
        src/imufusion/imufusion.cpp)

add_executable(${PROJECT_NAME} ${SOURCE})
target_include_directories(${PROJECT_NAME} PRIVATE
        ${PROJECT_SOURCE_DIR}/src
        ${PROJECT_SOURCE_DIR}/ThirdParty/eigen-3.3.9)

# find_package(OpenMP REQUIRED)
# if(OpenMP_CXX_FOUND)
#     message(STATUS "Found OpenMP")
#     target_link_libraries(${PROJECT_NAME} OpenMP::OpenMP_CXX)
# endif()

# yaml-cpp-0.7.0
add_subdirectory(ThirdParty/yaml-cpp-0.7.0)
target_link_libraries(${PROJECT_NAME} yaml-cpp)


set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1)
target_link_libraries(${PROJECT_NAME}
        absl::strings
        absl::str_format
        absl::time)
