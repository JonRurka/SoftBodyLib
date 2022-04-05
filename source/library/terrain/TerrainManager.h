#pragma once

#include "terrain.h"

namespace SoftBodyLib {
	class SimpleTerrainManager : public TerrainManager_Base
	{
	public:
		SimpleTerrainManager(Collisions_Base* col);


		// Terrain properties
		std::string getTerrainName();

		// Simulation
		virtual void			setGravity(glm::vec3 value) = 0;
		virtual glm::vec3		getGravity() = 0;
		float					GetHeightAt(float x, float z);
		glm::vec3				GetNormalAt(float x, float y, float z);
		glm::vec3				getMaxTerrainSize();
		AxisAlignedBox			getTerrainCollisionAAB();


		// Utility
		void                    LoadPredefinedActors();
		bool                    HasPredefinedActors();

	private:
		void initTerrainCollisions();
		void initObjects();
		void loadTerrainObjects();

		glm::vec3 m_gravity;
	};
}