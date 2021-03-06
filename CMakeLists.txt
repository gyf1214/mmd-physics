cmake_minimum_required(VERSION 3.2)
project(mmd-physics)

set(EXECUTABLE_OUTPUT_PATH bin)
set(LIBRARY_OUTPUT_PATH lib)

set(LIB_TARGET mmd-physics)

aux_source_directory(src SRCS)
aux_source_directory(src/ik SRCS)
set(EXAMPLE_SRC example)

include_directories(include)
include_directories(../mmd-tools/include)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=c++11 -DMMDShared")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -D__FILENAME__='\"$(subst ${CMAKE_SOURCE_DIR}/src/,,$(abspath $<))\"'")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -DNDEBUG -fvisibility=hidden")

add_library(${LIB_TARGET} SHARED ${SRCS})

find_package(Bullet REQUIRED)
target_link_libraries(${LIB_TARGET} ${BULLET_LIBRARIES})
include_directories(${BULLET_INCLUDE_DIRS})
