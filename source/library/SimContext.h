#pragma once

#include "physics/Physics.h"
#include "resources/resources.h"
#include "terrain/terrain.h"
#include "resources/resources.h"
#include "C_Variables.h"

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

        Actor* SpawnActor(ActorSpawnRequest& rq, FileBuilder* file_builder);
        //void                ModifyActor(ActorModifyRequest& rq);
        void                DeleteActor(Actor* actor);
        void                UpdateActors(float dt);
        ActorManager* GetActorManager() { return &m_actor_manager; }
        SimState GetSimState() { return m_sim_state; }
        void SetSimState(SimState newState) { m_sim_state = newState; }

        bool           IsSimulationPaused();
        void           SetSimulationPaused(bool v);

        C_Vec3 DoTest(SoftBodyLib::ActorSpawnRequest rq) {
            return C_Vec3::To(rq.asr_position);
        }


    private:


        // Actors (physics)
        ActorManager         m_actor_manager;
        TerrainManager_Base* g_sim_terrain{ nullptr };
        Collisions_Base*     g_collisions{ nullptr };
        SimState             m_sim_state{ SimState ::OFF};

        Actor* m_last_spawned_actor = nullptr;     //!< Last actor spawned by user and still alive.

	};
}





