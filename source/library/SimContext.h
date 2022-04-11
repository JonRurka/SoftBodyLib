#pragma once

#include "physics/Physics.h"
#include "resources/resources.h"
#include "terrain/terrain.h"
#include "resources/resources.h"

namespace SoftBodyLib {
	using namespace Util;

	class SimContext {
    public:

        enum class SimState
        {
            OFF,
            RUNNING,
            PAUSED,
            EDITOR_MODE //!< Hacky, but whatever... added by Ulteq, 2016
        };

        // ----------------------------
        // Terrain

        bool                LoadTerrain(TerrainManager_Base::TerrainSettings terrn_settings);
        void                UnloadTerrain();

        // ----------------------------
        // Actors

        Actor* SpawnActor(ActorSpawnRequest& rq, FileBuilder file_builder);
        //void                ModifyActor(ActorModifyRequest& rq);
        void                DeleteActor(Actor* actor);
        void                UpdateActors();
        ActorManager* GetActorManager() { return &m_actor_manager; }
        SimState GetSimState() { return m_sim_state; }
        void SetSimState(SimState newState) { m_sim_state = newState; }

    private:


        // Actors (physics)
        ActorManager         m_actor_manager;
        TerrainManager_Base* g_sim_terrain;
        Collisions_Base*     g_collisions;
        SimState             m_sim_state;

        Actor* m_last_spawned_actor = nullptr;     //!< Last actor spawned by user and still alive.

	};
}

extern "C"
{
    void* SimContext_New();

    bool SimContext_LoadTerrain(void* sim_context, void* terrain_mgr, void* collisions, float gravity);

    void SimContext_UnloadTerrain(void* sim_context);

    void* SimContext_SpawnActor(void* sim_context, SoftBodyLib::ActorSpawnRequest& rq, void* file_builder);

    void  SimContext_DeleteActor(void* sim_context, void* actor);

    void SimContext_ModifyActor(void* sim_context);

    void SimContext_UpdateActors(void* sim_context);

    void* SimContext_GetActorManager(void* sim_context);

    int SimContext_GetSimState(void* sim_context);
}

