#include "SimContext.h"

using namespace SoftBodyLib;

bool SoftBodyLib::SimContext::LoadTerrain(TerrainManager_Base::TerrainSettings terrn_settings)
{
	m_last_spawned_actor = nullptr;

	g_sim_terrain = TerrainManager_Base::LoadAndPrepareTerrain(terrn_settings);
	if (!g_sim_terrain)
	{
		return false;
	}
	

	return true;
}

void SoftBodyLib::SimContext::UnloadTerrain()
{
	if (g_sim_terrain != nullptr)
	{
		delete g_sim_terrain;
	}
}

Actor* SoftBodyLib::SimContext::SpawnActor(ActorSpawnRequest& rq, FileBuilder file_builder)
{
	bool predefined_on_terrain = rq.asr_origin == (ActorSpawnRequest::Origin)ActorSpawnRequest::DefaultOrigin::TERRN_DEF;
	std::shared_ptr<File> def = std::shared_ptr<File>(&file_builder.file);

	if (predefined_on_terrain)
	{
		// todo
	}

	if (def == nullptr)
	{
		return nullptr; // todo: report error.
	}

	// if (rq.asr_skin_entry != nullptr)

	Actor* fresh_actor = m_actor_manager.CreateActorInstance(rq, def);

	if (rq.asr_spawnbox == nullptr)
	{
		// Try to resolve collisions with other actors
		fresh_actor->resolveCollisions(50.0f, true /*m_player_actor == nullptr */ );
	}

	if (rq.asr_terrn_machine)
	{
		fresh_actor->ar_driveable = ActorType::MACHINE;
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

void SoftBodyLib::SimContext::UpdateActors()
{
	m_actor_manager.UpdateActors(nullptr);
}
