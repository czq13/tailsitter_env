cmake_minimum_required (VERSION 2.6)

project (tailsitter_env)

find_package(gazebo REQUIRED)

add_definitions(-DECL_STANDALONE=1)
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

include_directories(./fw)
#include_directories(./fw/mathlib)
include_direcotries(./mathlib)
include_directories(./matrix)
include_directories(/home/czq/chWorkspace/build_env/lib/ecl)

file(GLOB FW_SRC "./fw/*.cpp")
file(GLOB MATHLIB_SRC "./fw/mathlib/*.cpp")
file(GLOB SRC "./*.cpp")

add_executable(tailsitter_env ${SRC} ${FW_SRC} ${MATHLIB_SRC})
target_link_libraries(tailsitter_env ${GAZEBO_LIBRARIES} pthread)