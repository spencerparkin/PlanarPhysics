# CMakeLists.txt for PlanePhysics project

cmake_minimum_required(VERSION 3.27)

project(PlanarPhysics)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(PLANAR_PHYSICS_DEMO "Compile the demo in addition to the library." ON)

add_subdirectory(Engine)

if(PLANAR_PHYSICS_DEMO)
	add_subdirectory(Demo)
endif()