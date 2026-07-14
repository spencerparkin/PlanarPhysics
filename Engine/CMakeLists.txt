# CMakeLists.txt for planar physics engine library.

set(PLANAR_PHYSICS_ENGINE_SOURCES
	Source/PlanarPhysics/BoxTree.cpp
	Source/PlanarPhysics/BoxTree.h
	Source/PlanarPhysics/CollisionHandler.cpp
	Source/PlanarPhysics/CollisionHandler.h
	Source/PlanarPhysics/Common.h
	Source/PlanarPhysics/Engine.cpp
	Source/PlanarPhysics/Engine.h
	Source/PlanarPhysics/PlanarObject.cpp
	Source/PlanarPhysics/PlanarObject.h
	Source/PlanarPhysics/PlanarObjects/Ball.cpp
	Source/PlanarPhysics/PlanarObjects/Ball.h
	Source/PlanarPhysics/PlanarObjects/RigidBody.cpp
	Source/PlanarPhysics/PlanarObjects/RigidBody.h
	Source/PlanarPhysics/PlanarObjects/Wall.cpp
	Source/PlanarPhysics/PlanarObjects/Wall.h
	Source/PlanarPhysics/Math/GeometricAlgebra/Multivector2D.cpp
	Source/PlanarPhysics/Math/GeometricAlgebra/Multivector2D.h
	Source/PlanarPhysics/Math/GeometricAlgebra/PScalar2D.cpp
	Source/PlanarPhysics/Math/GeometricAlgebra/PScalar2D.h
	Source/PlanarPhysics/Math/GeometricAlgebra/Rotor2D.cpp
	Source/PlanarPhysics/Math/GeometricAlgebra/Rotor2D.h
	Source/PlanarPhysics/Math/GeometricAlgebra/Vector2D.cpp
	Source/PlanarPhysics/Math/GeometricAlgebra/Vector2D.h
	Source/PlanarPhysics/Math/Utilities/BoundingBox.cpp
	Source/PlanarPhysics/Math/Utilities/BoundingBox.h
	Source/PlanarPhysics/Math/Utilities/ConvexPolygon.cpp
	Source/PlanarPhysics/Math/Utilities/ConvexPolygon.h
	Source/PlanarPhysics/Math/Utilities/Line.cpp
	Source/PlanarPhysics/Math/Utilities/Line.h
	Source/PlanarPhysics/Math/Utilities/LineSegment.cpp
	Source/PlanarPhysics/Math/Utilities/LineSegment.h
	Source/PlanarPhysics/Math/Utilities/Random.cpp
	Source/PlanarPhysics/Math/Utilities/Random.h
	Source/PlanarPhysics/Math/Utilities/Ray.cpp
	Source/PlanarPhysics/Math/Utilities/Ray.h
	Source/PlanarPhysics/Math/Utilities/Transform.cpp
	Source/PlanarPhysics/Math/Utilities/Transform.h
	Source/PlanarPhysics/CollisionHandlers/BallBallCH.cpp
	Source/PlanarPhysics/CollisionHandlers/BallBallCH.h
	Source/PlanarPhysics/CollisionHandlers/BallWallCH.cpp
	Source/PlanarPhysics/CollisionHandlers/BallWallCH.h
	Source/PlanarPhysics/CollisionHandlers/RBodyRBodyCH.cpp
	Source/PlanarPhysics/CollisionHandlers/RBodyRBodyCH.h
	Source/PlanarPhysics/CollisionHandlers/RigidBodyBallCH.cpp
	Source/PlanarPhysics/CollisionHandlers/RigidBodyBallCH.h
	Source/PlanarPhysics/CollisionHandlers/RigidBodyWallCH.cpp
	Source/PlanarPhysics/CollisionHandlers/RigidBodyWallCH.h
)

add_library(PlanarPhysics STATIC ${PLANAR_PHYSICS_ENGINE_SOURCES})

target_include_directories(PlanarPhysics PUBLIC Source)