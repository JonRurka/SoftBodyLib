set(module "PhysicsFlex")

set(${module}_headers
    FlexObj.h
)

set(${module}_sources
    FlexObj.cpp
)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})
