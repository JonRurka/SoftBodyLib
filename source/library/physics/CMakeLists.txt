set(module "Physics")

set(${module}_headers
    Actor.h
    ApproxMath.h
    Physics.h
    SimConstants.h
    SimData.h
	ActorManager.h
	ActorSpawner.h
)

set(${module}_sources
    SimData.cpp
    Actor.cpp
	ActorManager.cpp
	ActorSpawner.cpp
)

add_subdirectory(collision)
add_subdirectory(flex)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})

target_link_libraries(${module} PhysicsCollision)
target_link_libraries(${module} PhysicsFlex)
