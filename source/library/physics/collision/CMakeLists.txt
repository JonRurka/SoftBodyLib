set(module "PhysicsCollision")

set(${module}_headers
    CartesianToTriangleTransform.h
	Collisions_Base.h
    Collisions.h
    DynamicCollisions.h
    PointColDetector.h
    Triangle.h
)

set(${module}_sources
    Collisions.cpp
    DynamicCollisions.cpp
    PointColDetector.cpp
)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})
