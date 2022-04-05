#include "terrain.h"

using namespace SoftBodyLib;

TerrainManager_Base* SoftBodyLib::TerrainManager_Base::LoadAndPrepareTerrain(TerrainSettings terrn_settings)
{

	terrn_settings.terrain_mgr->setGravity(glm::vec3(0, terrn_settings.gravity, 0));

	//terrn_mgr->m_geometry_manager = new TerrainGeometryManager(terrn_mgr.get());


	terrn_settings.terrain_mgr->initObjects();


	/*if (!terrn_mgr->m_geometry_manager->InitTerrain(terrn_mgr->m_def.ogre_ter_conf_filename))
	{
		return nullptr; // Error already reported
	}*/

	terrn_settings.terrain_mgr->m_collisions = terrn_settings.collisions;

	terrn_settings.terrain_mgr->loadTerrainObjects(); // *.tobj files

	terrn_settings.terrain_mgr->initTerrainCollisions();

	terrn_settings.terrain_mgr->m_collisions->finishLoadingTerrain();


	terrn_settings.terrain_mgr->LoadPredefinedActors();

	return terrn_settings.terrain_mgr;
}

SoftBodyLib::TerrainManager_Base::TerrainManager_Base(Collisions_Base* col) :
	m_collisions(col)
{
}




// ***************************
// ****** SimpleTerrainManager
// ***************************

SoftBodyLib::SimpleTerrainManager::SimpleTerrainManager(Collisions_Base* col) :
	TerrainManager_Base(col), m_gravity(0)
{

}

std::string SoftBodyLib::SimpleTerrainManager::getTerrainName()
{
	return "BasicTerrain";
}

void SoftBodyLib::SimpleTerrainManager::setGravity(glm::vec3 value)
{
	m_gravity = value;
}

glm::vec3 SoftBodyLib::SimpleTerrainManager::getGravity()
{
	return m_gravity;
}

float SoftBodyLib::SimpleTerrainManager::GetHeightAt(float x, float z)
{
	return 0.0f;
}

glm::vec3 SoftBodyLib::SimpleTerrainManager::GetNormalAt(float x, float y, float z)
{
	return glm::vec3(0, 1, 0);
}

glm::vec3 SoftBodyLib::SimpleTerrainManager::getMaxTerrainSize()
{
	return glm::vec3(100, 100, 100);
}

AxisAlignedBox SoftBodyLib::SimpleTerrainManager::getTerrainCollisionAAB()
{
	return AxisAlignedBox(glm::vec3(-50, -50, -50), glm::vec3(50, 50, 50));
}

void SoftBodyLib::SimpleTerrainManager::LoadPredefinedActors()
{
}

bool SoftBodyLib::SimpleTerrainManager::HasPredefinedActors()
{
	return false;
}

void SoftBodyLib::SimpleTerrainManager::initTerrainCollisions()
{
}

void SoftBodyLib::SimpleTerrainManager::initObjects()
{
}

void SoftBodyLib::SimpleTerrainManager::loadTerrainObjects()
{
}
