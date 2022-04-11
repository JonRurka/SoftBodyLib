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

SoftBodyLib::SimpleTerrainManager::SimpleTerrainManager() :
	TerrainManager_Base(0), m_gravity(0)
{
}

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
	return	1.2f;
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


////////////////////
/// C INTERFACE

// #### TerrainManager_Base



std::string TerrainManager_Base_getTerrainName(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->getTerrainName();
}

void* TerrainManager_Base_GetCollisions(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->GetCollisions();
}

void TerrainManager_Base_SetCollisions(void* handle, void* col)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	terr->SetCollisions((Collisions_Base*)col);
}

void TerrainManager_Base_setGravity(void* handle, glm::vec3 value)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;
	terr->setGravity(value);
}

glm::vec3 TerrainManager_Base_getGravity(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->getGravity();
}

float TerrainManager_Base_GetHeightAt(void* handle, float x, float z)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->GetHeightAt(x, z);
}

glm::vec3 TerrainManager_Base_GetNormalAt(void* handle, float x, float y, float z)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->GetNormalAt(x, y, z);
}

glm::vec3 TerrainManager_Base_getMaxTerrainSize(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->getMaxTerrainSize();
}

void* TerrainManager_Base_getTerrainCollisionAAB(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return &terr->getTerrainCollisionAAB();
}


// #### SimpleTerrainManager

void* SimpleTerrainManager_New()
{
	return new SimpleTerrainManager();
}

void* SimpleTerrainManager_New_col(void* col)
{
	return new SimpleTerrainManager((Collisions_Base*)col);
}
