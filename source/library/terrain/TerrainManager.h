#pragma once

#include "terrain.h"

namespace SoftBodyLib {
	class SimpleTerrainManager : public TerrainManager_Base
	{
	public:
		SimpleTerrainManager();
		SimpleTerrainManager(Collisions_Base* col);


		// Terrain properties
		std::string getTerrainName();

		// Simulation
		virtual void			setGravity(glm::vec3 value);
		virtual glm::vec3		getGravity();
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

extern "C" {
	void* SimpleTerrainManager_New();

	void* SimpleTerrainManager_New_col(void* col);
}
