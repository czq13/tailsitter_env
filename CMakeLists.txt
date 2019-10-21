cmake_minimum_required (VERSION 2.6)

project (tailsitter_env)

find_package(gazebo REQUIRED)

add_definitions(-DECL_STANDALONE=1)

include_directories(./fw)
include_directories(./fw/mathlib)
include_directories(./matrix)
include_directories(/home/czq/chWorkspace/build_env/lib/ecl)

file(GLOB FW_SRC "./fw/*.cpp")
file(GLOB MATHLIB_SRC "./fw/mathlib/*.cpp")

add_executable(tailsitter_env tailsitter_env.cpp ${FW_SRC} ${MATHLIB_SRC})
