set(module "Utils")

set(${module}_headers
    utils.h
    AxisAlignedBox.h
    Math.h
    Plane.h
    Ray.h
    Sphere.h
    ZeroedMemoryAllocator.h
	BitFlags.h
)

set(${module}_sources
    AxisAlignedBox.cpp
    Math.cpp
    Plane.cpp
    Ray.cpp
    Sphere.cpp
)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})
