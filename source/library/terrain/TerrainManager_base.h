#pragma once

#include "physics/Physics.h"
#include "terrain.h"

namespace SoftBodyLib {

	class TerrainManager_Base
	{
	public:

		struct TerrainSettings {
			TerrainManager_Base* terrain_mgr;
			Collisions_Base* collisions;
			float gravity;
		};

		static TerrainManager_Base* LoadAndPrepareTerrain(TerrainSettings terrn_settings);

		TerrainManager_Base(Collisions_Base* col);

		// Terrain properties
		virtual std::string getTerrainName() = 0;

		// Subsystems
		Collisions_Base* GetCollisions() { return m_collisions; }
		void SetCollisions(Collisions_Base* col) { m_collisions = col; }

		// Simulation
		virtual void			setGravity(glm::vec3 value) = 0;
		virtual glm::vec3		getGravity() = 0;
		virtual float           GetHeightAt(float x, float z) = 0;
		virtual glm::vec3		GetNormalAt(float x, float y, float z) = 0;
		virtual glm::vec3		getMaxTerrainSize() = 0;
		virtual AxisAlignedBox	getTerrainCollisionAAB() = 0;


		// Utility
		virtual void                    LoadPredefinedActors() = 0;
		virtual bool                    HasPredefinedActors() = 0;

	protected:
		virtual void initTerrainCollisions() = 0;
		virtual void initObjects() = 0;
		virtual void loadTerrainObjects() = 0;

		Collisions_Base* m_collisions;

	};
}

