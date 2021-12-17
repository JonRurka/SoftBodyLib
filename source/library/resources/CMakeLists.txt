set(module "Resources")

set(${module}_headers
    resources.h
	FileBuilder.h
)

set(${module}_sources
	FileBuilder.cpp
)

add_subdirectory(rig_def_fileformat)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})
target_link_libraries(${module} RigDefFile)