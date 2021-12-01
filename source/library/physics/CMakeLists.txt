set(module "Physics")

set(${module}_headers
    Actor.h
    ApproxMath.h
    Physics.h
    SimConstants.h
    SimData.h
)

set(${module}_sources
    Actor.cpp
)

add_subdirectory(collision)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})
target_link_libraries(${module} PhysicsCollision)
