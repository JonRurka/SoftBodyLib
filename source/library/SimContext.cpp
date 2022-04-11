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


////////////////////
/// C INTERFACE

void* SimContext_New()
{
	return new SimContext();
}

float SimContext_Test(void* sim_context, float a, float b)
{
	SimContext* cont = (SimContext*)sim_context;

	return cont->DoTest(a, b);
}

bool SimContext_LoadTerrain(void* sim_context, void* terrain_mgr, void* collisions, float gravity)
{
	SimContext* cont = (SimContext*)sim_context;

	TerrainManager_Base* terr = (TerrainManager_Base*)terrain_mgr;
	Collisions_Base* col = (Collisions_Base*)collisions;

	TerrainManager_Base::TerrainSettings terrn_settings;
	terrn_settings.terrain_mgr = terr;
	terrn_settings.collisions = col;
	terrn_settings.gravity = gravity;

	return cont->LoadTerrain(terrn_settings);
}

void SimContext_UnloadTerrain(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	cont->UnloadTerrain();
}

void* SimContext_SpawnActor(void* sim_context, SoftBodyLib::ActorSpawnRequest& rq, void* file_builder)
{
	SimContext* cont = (SimContext*)sim_context;

	FileBuilder builder = *((FileBuilder*)file_builder);

	return cont->SpawnActor(rq, builder);
}

void SimContext_DeleteActor(void* sim_context, void* actor)
{
	SimContext* cont = (SimContext*)sim_context;

	Actor* act = (Actor*)actor;

	cont->DeleteActor(act);
}

void SimContext_ModifyActor(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	// todo
}

void SimContext_UpdateActors(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	cont->UpdateActors();
}

void* SimContext_GetActorManager(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	return cont->GetActorManager();
}

int SimContext_GetSimState(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	return (int)cont->GetSimState();
}
