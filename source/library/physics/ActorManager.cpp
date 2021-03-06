#include "ActorManager.h"
#include "Logger.h"

using namespace SoftBodyLib;

static int m_actor_counter = 0;

ActorManager::ActorManager() :
	m_dt_remainder(0.0f),
	m_forced_awake(false),
	m_physics_steps(2000),
	m_simulation_speed(1.0f)
{
	// Create worker thread (used for physics calculations)
	// TODO: m_sim_thread_pool = std::unique_ptr<ThreadPool>(new ThreadPool(1));
}

ActorManager::~ActorManager()
{
	//this->SyncWithSimThread(); // Wait for sim task to finish
}

void ActorManager::SetupActor(Actor* actor, ActorSpawnRequest rq, std::shared_ptr<File> def)
{
	ActorSpawner spawner;
	spawner.Setup(actor, def, rq.asr_position);

	/* Setup modules */
	spawner.AddModule(def->root_module);

	spawner.SpawnActor();

	actor->ar_initial_node_positions.resize(actor->ar_num_nodes);
	actor->ar_initial_beam_defaults.resize(actor->ar_num_beams);
	actor->ar_initial_node_masses.resize(actor->ar_num_nodes);

	actor->UpdateBoundingBoxes(); // (records the unrotated dimensions for 'veh_aab_size')
	
	// Apply spawn position & spawn rotation
	for (int i = 0; i < actor->ar_num_nodes; i++)
	{
		actor->ar_nodes[i].AbsPosition = rq.asr_position + rq.asr_rotation * (actor->ar_nodes[i].AbsPosition - rq.asr_position);
		actor->ar_nodes[i].RelPosition = actor->ar_nodes[i].AbsPosition - actor->ar_origin;
	}


	/* Place correctly */
	if (spawner.GetMemoryRequirements().num_fixes == 0)
	{
		glm::vec3 vehicle_position = rq.asr_position;

		// check if over-sized
		actor->UpdateBoundingBoxes();
		vehicle_position.x += vehicle_position.x - actor->ar_bounding_box.getCenter().x;
		vehicle_position.z += vehicle_position.z - actor->ar_bounding_box.getCenter().z;

		float miny = 0.0f;

		// if (!actor->m_preloaded_with_terrain)
		miny = vehicle_position.y;

		if (rq.asr_spawnbox != nullptr)
		{
			miny = rq.asr_spawnbox->relo.y + rq.asr_spawnbox->center.y;
		}

		if (rq.asr_free_position)
			actor->resetPosition(vehicle_position, true);
		else
			actor->resetPosition(vehicle_position.x, vehicle_position.z, true, miny);

		if (rq.asr_spawnbox != nullptr)
		{
			bool inside = true;

			for (int i = 0; i < actor->ar_num_nodes; i++)
			{
				// TODO: inside = (inside && App::GetSimTerrain()->GetCollisions()->isInside(actor->ar_nodes[i].AbsPosition, rq.asr_spawnbox, 0.2f));
			}

			if (!inside)
			{
				glm::vec3 gpos = glm::vec3(vehicle_position.x, 0.0f, vehicle_position.z);

				gpos -= rq.asr_rotation * glm::vec3((rq.asr_spawnbox->hi.x - rq.asr_spawnbox->lo.x + actor->ar_bounding_box.getMaximum().x - actor->ar_bounding_box.getMinimum().x) * 0.6f, 0.0f, 0.0f);

				actor->resetPosition(gpos.x, gpos.z, true, miny);
			}
		}
	}
	else
	{
		actor->resetPosition(rq.asr_position, true);
	}
	actor->UpdateBoundingBoxes();

	//compute final mass
	actor->RecalculateNodeMasses(actor->m_dry_mass);
	actor->ar_initial_total_mass = actor->m_total_mass;
	for (int i = 0; i < actor->ar_num_nodes; i++)
	{
		actor->ar_initial_node_masses[i] = actor->ar_nodes[i].mass;
	}

	//compute node connectivity graph
	actor->calcNodeConnectivityGraph();

	actor->UpdateBoundingBoxes();
	actor->calculateAveragePosition();

	// calculate minimum camera radius
	actor->calculateAveragePosition();
	for (int i = 0; i < actor->ar_num_nodes; i++)
	{
		// TODO:
		//Real dist = actor->ar_nodes[i].AbsPosition.squaredDistance(actor->m_avg_node_position);
		//if (dist > actor->m_min_camera_radius)
		//{
		//	actor->m_min_camera_radius = dist;
		//}
	}
	// actor->m_min_camera_radius = std::sqrt(actor->m_min_camera_radius) * 1.2f; // twenty percent buffer

	// fix up submesh collision model
	//std::string subMeshGroundModelName = "";// TODO: spawner.GetSubmeshGroundmodelName();
	//if (!subMeshGroundModelName.empty())
	//{
	actor->ar_submesh_ground_model = ((Collisions*)collision)->getGroundModelByString("concrete"); //nullptr;// TODO: App::GetSimTerrain()->GetCollisions()->getGroundModelByString(subMeshGroundModelName);
	if (!actor->ar_submesh_ground_model)
	{
		Logger::LogError("Setup Actor", "No Ground Model for actor!");
		actor->ar_submesh_ground_model = nullptr;//collision->defaultgm;
	}
	//}

	// Set beam defaults
	for (int i = 0; i < actor->ar_num_beams; i++)
	{
		actor->ar_beams[i].initial_beam_strength = actor->ar_beams[i].strength;
		actor->ar_beams[i].default_beam_deform = actor->ar_beams[i].minmaxposnegstress;
		actor->ar_initial_beam_defaults[i] = std::make_pair(actor->ar_beams[i].k, actor->ar_beams[i].d);
	}

	actor->m_spawn_rotation = actor->getRotation();

	// TRIGGER_EVENT(SE_GENERIC_NEW_TRUCK, actor->ar_instance_id);

	actor->ar_state = ActorState::LOCAL_SIMULATED;
	actor->ar_physics_paused = false;

	Logger::LogInfo("Actor Spawner", " ===== DONE LOADING VEHICLE");
}

Actor* ActorManager::CreateActorInstance(ActorSpawnRequest rq, std::shared_ptr<File> def)
{
	Actor* actor = nullptr;

	try {
		actor = new Actor(m_actor_counter++, static_cast<int>(m_actors.size()), def, rq, this, terrain);
		//actor->setUsedSkin(rq.asr_skin_entry);

		this->SetupActor(actor, rq, def);

		m_actors.push_back(actor);
	}
	catch (std::exception& e)
	{
		Logger::LogError("ActorManager::CreateActorInstance", e.what());
	}
		
	return actor;
}

void SoftBodyLib::ActorManager::CleanUpSimulation()
{
}

void SoftBodyLib::ActorManager::UpdateSleepingState(Actor* player_actor, float dt)
{
}

void SoftBodyLib::ActorManager::DeleteActorInternal(Actor* actor)
{
	if (actor == 0)
		return;

	this->SyncWithSimThread();

	m_actors.erase(std::remove(m_actors.begin(), m_actors.end(), actor), m_actors.end());

	delete actor;

	// Update actor indices
	for (unsigned int i = 0; i < m_actors.size(); i++)
		m_actors[i]->ar_vector_index = i;
}

std::vector<Actor*> SoftBodyLib::ActorManager::GetLocalActors()
{
	std::vector<Actor*> actors;
	for (auto actor : m_actors)
	{
		actors.push_back(actor);
	}

	return actors;
}

void SoftBodyLib::ActorManager::UpdateActors(Actor* player_actor, float g_dt)
{
	if (m_simulation_paused)
	{
		m_simulation_time = 0.f;
		//Logger::LogDebug("ActorManager::UpdateActors", "SIM PAUSED");
	}
	else
	{
		m_simulation_time = g_dt;
		//Logger::LogDebug("ActorManager::UpdateActors", "Sime Time: " + std::to_string(m_simulation_time));
	}




	float dt = m_simulation_time;

	// do not allow dt > 1/20
	dt = std::min(dt, 1.0f / 20.0f);

	dt *= m_simulation_speed;

	dt += m_dt_remainder;
	m_physics_steps = dt / PHYSICS_DT;

	//Logger::LogDebug("ActorManager::UpdateActors", "m_physics_steps: " + std::to_string(m_physics_steps));

	if (m_physics_steps == 0)
	{
		//Logger::LogDebug("ActorManager::UpdateActors", "m_physics_steps == 0");
		return;
	}

	m_dt_remainder = dt - (m_physics_steps * PHYSICS_DT);
	dt = PHYSICS_DT * m_physics_steps;

	this->SyncWithSimThread();

	// this->UpdateSleepingState(player_actor, dt);

	for (auto actor : m_actors)
	{
		// todo: engine stuff

		// skid marks

		// dashboard

		// flare state
	}

	this->UpdatePhysicsSimulation();

	m_total_sim_time += dt;

	// join
}

void SoftBodyLib::ActorManager::SyncWithSimThread()
{
}

void SoftBodyLib::ActorManager::UpdatePhysicsSimulation()
{
	//Logger::LogDebug("ActorManager::UpdatePhysicsSimulation", "UpdatePhysicsSimulation");
	for (auto actor : m_actors)
	{
		actor->UpdatePhysicsOrigin();
	}
	//Logger::LogDebug("ActorManager::UpdatePhysicsSimulation", "finished UpdatePhysicsOrigin");
	for (int i = 0; i < m_physics_steps; i++)
	{
		for (auto actor : m_actors)
		{
			if (actor->ar_update_physics = actor->CalcForcesEulerPrepare(i == 0))
			{
				//Logger::LogDebug("ActorManager::UpdatePhysicsSimulation", "CalcForcesEulerPrepare was true");
				actor->CalcForcesEulerCompute(1 == 0, m_physics_steps);
			}
		}

		for (auto actor : m_actors)
		{
			if (actor->ar_update_physics)
			{
				actor->CalcBeamsInterActor();
			}
		}

		for (auto actor : m_actors)
		{
			if (actor->m_inter_point_col_detector != nullptr &&
				actor->ar_update_physics)
			{
				actor->m_inter_point_col_detector->UpdateInterPoint();
				if (actor->ar_collision_relevant)
				{
					ResolveInterActorCollisions(PHYSICS_DT,
						*actor->m_inter_point_col_detector,
						actor->ar_num_collcabs,
						actor->ar_collcabs,
						actor->ar_cabs,
						actor->ar_inter_collcabrate,
						actor->ar_nodes,
						actor->ar_collision_range,
						*actor->ar_submesh_ground_model);
				}
			}
		}
	}
	for (auto actor : m_actors)
	{
		actor->m_ongoing_reset = false;
		if (actor->ar_update_physics && m_physics_steps > 0)
		{
			// camera forces
			
			actor->calculateLocalGForces();
			actor->calculateAveragePosition();
			actor->m_avg_node_velocity = actor->m_avg_node_position - actor->m_avg_node_position_prev;
			actor->m_avg_node_velocity /= (m_physics_steps * PHYSICS_DT);
			actor->m_avg_node_position_prev = actor->m_avg_node_position;
			actor->ar_top_speed = std::max(actor->ar_top_speed, glm::length(actor->ar_nodes[0].Velocity));

		}
	}
}

