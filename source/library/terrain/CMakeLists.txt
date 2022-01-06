set(module "Terrain")

set(${module}_headers
	terrain.h
	TerrainManager_base.h
	TerrainManager.h
	
)

set(${module}_sources
	TerrainManager.cpp
)

add_library(${module} STATIC ${${module}_headers} ${${module}_sources})
