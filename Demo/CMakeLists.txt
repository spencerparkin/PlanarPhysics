# CMakeLists.txt for planar physics demo.

set(PLANAR_PHYSICS_DEMO_SOURCES
	Source/App.cpp
	Source/App.h
	Source/DrawHelper.cpp
	Source/DrawHelper.h
	Source/FramerateTracker.cpp
	Source/FramerateTracker.h
	Source/Main.cpp
	Source/Main.h
	Source/RenderObject.cpp
	Source/RenderObject.h
	Source/RenderObjects/RenderBall.cpp
	Source/RenderObjects/RenderBall.h
	Source/RenderObjects/RenderRigidBody.cpp
	Source/RenderObjects/RenderRigidBody.h
	Source/RenderObjects/RenderWall.cpp
	Source/RenderObjects/RenderWall.h
)

include(FetchContent)

FetchContent_Declare(
	SDL3
    GIT_REPOSITORY https://github.com/libsdl-org/SDL.git
    GIT_TAG release-3.2.16
)

FetchContent_MakeAvailable(SDL3)

add_executable(PlanarPhysicsDemo ${PLANAR_PHYSICS_DEMO_SOURCES})

target_link_libraries(PlanarPhysicsDemo PRIVATE
	PlanarPhysics
	SDL3::SDL3
)

target_include_directories(PlanarPhysicsDemo PRIVATE Source)