#include "SimContext.h"
#include "Logger.h"

using namespace SoftBodyLib;

bool SoftBodyLib::SimContext::LoadTerrain(TerrainManager_Base::TerrainSettings terrn_settings)
{
	Logger::LogInfo("SimContext::LoadTerrain", "Loading Terrain: " + terrn_settings.terrain_mgr->getTerrainName());

	m_last_spawned_actor = nullptr;

	g_sim_terrain = TerrainManager_Base::LoadAndPrepareTerrain(terrn_settings);
	if (!g_sim_terrain)
	{
		return false;
	}

	
	g_sim_terrain = terrn_settings.terrain_mgr;
	g_collisions = terrn_settings.collisions;
	
	m_actor_manager.SetTerrainManager(g_sim_terrain);
	m_actor_manager.SetCollision(g_collisions);
	

	return true;
}

void SoftBodyLib::SimContext::UnloadTerrain()
{
	if (g_sim_terrain != nullptr)
	{
		delete g_sim_terrain;
	}
}

Actor* SoftBodyLib::SimContext::SpawnActor(ActorSpawnRequest& rq, FileBuilder* file_builder)
{
	Actor* fresh_actor = nullptr;

	//return fresh_actor;
	try {
		bool predefined_on_terrain = rq.asr_origin == (ActorSpawnRequest::Origin)ActorSpawnRequest::DefaultOrigin::TERRN_DEF;
		std::shared_ptr<File> def = std::shared_ptr<File>(&file_builder->file);

		if (predefined_on_terrain)
		{
			// todo
		}

		if (def == nullptr)
		{
			Logger::LogError("SimContext::SpawnActor", "File Definition was null!");
			return nullptr;
		}

		// if (rq.asr_skin_entry != nullptr)

		fresh_actor = m_actor_manager.CreateActorInstance(rq, def);

		if (rq.asr_spawnbox == nullptr)
		{
			// Try to resolve collisions with other actors
			fresh_actor->resolveCollisions(50.0f, true /*m_player_actor == nullptr */);
		}

		if (rq.asr_terrn_machine)
		{
			fresh_actor->ar_driveable = ActorType::MACHINE;
		}
	}
	catch (std::exception& e)
	{
		Logger::LogError("ActorManager::SpawnActor", e.what());
	}
	return fresh_actor;
}

void SoftBodyLib::SimContext::DeleteActor(Actor* actor)
{
	if (actor == m_last_spawned_actor)
	{
		m_last_spawned_actor = nullptr;
	}

	// Find linked actors and un-tie if tied
	auto linked_actors = actor->getAllLinkedActors();
	for (auto actorx : m_actor_manager.GetLocalActors())
	{
		// todo
	}

	m_actor_manager.DeleteActorInternal(actor);
}

void SoftBodyLib::SimContext::UpdateActors(float dt)
{
	m_actor_manager.UpdateActors(nullptr, dt);
}

bool SoftBodyLib::SimContext::IsSimulationPaused()
{
	return m_actor_manager.IsSimulationPaused();
}

void SoftBodyLib::SimContext::SetSimulationPaused(bool v)
{
	m_actor_manager.SetSimulationPaused(v);
}

