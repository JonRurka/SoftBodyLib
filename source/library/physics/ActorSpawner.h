#pragma once

#include "Physics.h"


namespace SoftBodyLib {
	/// Builds and manages softbody actors (physics on background thread, networking)
	class ActorManager {
	public:

		ActorManager();
		~ActorManager();

		Actor* CreateActorInstance();





	private:

		void SetupActor(Actor* actor);




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
	};
}