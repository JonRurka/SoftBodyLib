set(module "RigDefFile")

set(${module}_headers
    rigdef.h
	RigDef_Prerequisites.h
	RigDef_File.h
	RigDef_Node.h
)

set(${module}_sources
	RigDef_File.cpp
	RigDef_Node.cpp
)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})