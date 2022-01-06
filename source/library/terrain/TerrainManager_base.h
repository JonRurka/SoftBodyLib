#pragma once

#include "physics/Physics.h"
#include "terrain.h"

namespace SoftBodyLib {

	class TerrainManager_Base
	{
	public:
		// Terrain properties
		virtual std::string getTerrainName() = 0;

		// Subsystems
		virtual Collisions_Base* GetCollisions() = 0;

		// Simulation
		virtual void			setGravity(float value) = 0;
		virtual float			getGravity() = 0;
		virtual float           GetHeightAt(float x, float z) = 0;
		virtual glm::vec3		GetNormalAt(float x, float y, float z) = 0;
		virtual glm::vec3		getMaxTerrainSize() = 0;
		virtual AxisAlignedBox	getTerrainCollisionAAB() = 0;
	};
}

