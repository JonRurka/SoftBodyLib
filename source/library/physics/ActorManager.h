#pragma once

#include "Physics.h"
#include "resources/resources.h"
#include "terrain/terrain.h"

#define PHYSICS_DT 0.0005f // fixed dt of 0.5 ms

namespace SoftBodyLib {
	/// Builds and manages softbody actors (physics on background thread, networking)
	class ActorManager {
	public:

		ActorManager();
		~ActorManager();

		Actor*			CreateActorInstance(ActorSpawnRequest rq, std::shared_ptr<File> def);
		void			UpdateActors(Actor* player_actor);
		void			SyncWithSimThread();
		void			UpdatePhysicsSimulation();

		void           SetTrucksForcedAwake(bool forced) { m_forced_awake = forced; };
		bool           AreTrucksForcedAwake() const { return m_forced_awake; }
		void           SetSimulationSpeed(float speed) { m_simulation_speed = std::max(0.0f, speed); };
		float          GetSimulationSpeed() const { return m_simulation_speed; };
		bool           IsSimulationPaused() const { return m_simulation_paused; }
		void           SetSimulationPaused(bool v) { m_simulation_paused = v; }
		float          GetTotalTime() const { return m_total_sim_time; }

		void           CleanUpSimulation(); //!< Call this after simulation loop finishes.

		void           UpdateSleepingState(Actor* player_actor, float dt);

		void           DeleteActorInternal(Actor* b); //!< Use `GameContext::DeleteActor()`


		std::vector<Actor*> GetActors() const { return m_actors; }
		std::vector<Actor*> GetLocalActors();

		void SetTerrainManager(TerrainManager_Base* terr) { terrain = terr; }
		TerrainManager_Base* GetTerrainManager() { return terrain; }

		void SetCollision(Collisions_Base* col) { collision = col; }
		Collisions_Base* GetCollision() { return collision; }

	private:

		
		void SetupActor(Actor* actor, ActorSpawnRequest rq, std::shared_ptr<File> def);




		// Physics
		std::vector<Actor*> m_actors;
		bool                m_forced_awake = false; //!< disables sleep counters
		int                 m_physics_steps = 0;
		float               m_dt_remainder = 0.f;   //!< Keeps track of the rounding error in the time step calculation
		float               m_simulation_speed = 1.f;   //!< slow motion < 1.0 < fast motion
		float               m_last_simulation_speed = 0.1f;  //!< previously used time ratio between real time (evt.timeSinceLastFrame) and physics time ('dt' used in calcPhysics)
		float               m_simulation_time = 0.f;   //!< Amount of time the physics simulation is going to be advanced
		bool                m_simulation_paused = false;
		float               m_total_sim_time = 0.f;

		TerrainManager_Base* terrain;
		Collisions_Base* collision;
	};
}

