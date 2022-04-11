#include "Actor.h"

using namespace SoftBodyLib;

static const glm::vec3 BOUNDING_BOX_PADDING(0.05f, 0.05f, 0.05f);

Actor::Actor(
	int actor_id
	, unsigned int vector_index
	, std::shared_ptr<File> def
	, ActorSpawnRequest rq
	, ActorManager* mngr
	, TerrainManager_Base* ter
) :   ar_instance_id(actor_id)
	, ar_vector_index(vector_index) 
	, m_avg_node_position_prev(rq.asr_position)
	, m_avg_node_position(rq.asr_position)
	, m_actor_manager(mngr)
	, m_terrain(ter)

{
}

Actor::~Actor() {

}

float Actor::getRotation()
{
	glm::vec3 dir = getDirection();

	return atan2(glm::dot(dir, glm::vec3(1, 0, 0)), glm::dot(dir, -glm::vec3(0, 0, 1)));
}

glm::vec3 Actor::getDirection()
{
	return glm::vec3(0); // TODO: ar_main_camera_dir_corr * this->GetCameraDir();
}

glm::vec3 Actor::getPosition()
{
	return m_avg_node_position; //the position is already in absolute position
}

void Actor::RecalculateNodeMasses(Real total)
{
	// TODO: make plugin callback for "RecalculateNodeMasses excluded" for tyre

	// reset
	for (int i = 0; i < ar_num_nodes; i++)
	{
		if (!ar_nodes[i].nd_tyre_node)
		{
			if (!ar_nodes[i].nd_loaded_mass)
			{
				ar_nodes[i].mass = 0;
			}
			else if (!ar_nodes[i].nd_override_mass)
			{
				ar_nodes[i].mass = m_load_mass / (float)m_masscount;
			}
		}
	}

	//average linear density
	Real len = 0.0f;
	for (int i = 0; i < ar_num_beams; i++)
	{
		if (ar_beams[i].bm_type != BEAM_VIRTUAL)
		{
			Real half_newlen = ar_beams[i].L / 2.0f;
			if (!ar_beams[i].p1->nd_tyre_node)
				len += half_newlen;
			if (!ar_beams[i].p2->nd_tyre_node)
				len += half_newlen;
		}
	}

	for (int i = 0; i < ar_num_beams; i++)
	{
		if (ar_beams[i].bm_type != BEAM_VIRTUAL)
		{
			Real half_mass = ar_beams[i].L * total / len / 2.0f;
			if (!ar_beams[i].p1->nd_tyre_node)
				ar_beams[i].p1->mass += half_mass;
			if (!ar_beams[i].p2->nd_tyre_node)
				ar_beams[i].p2->mass += half_mass;
		}
	}

	// TODO:
	
	//fix rope masses

	// Apply pre-defined cinecam node mass

	//update mass
	for (int i = 0; i < ar_num_nodes; i++)
	{
		// TODO: research where minimass_skip_loaded_nodes is set.

		if (!ar_nodes[i].nd_tyre_node &&
			!(m_definition->minimass_skip_loaded_nodes && ar_nodes[i].nd_loaded_mass) &&
			ar_nodes[i].mass < ar_minimass[i])
		{
			if (false) // diagnose truck mass
			{
				// Node '%d' mass(% f Kg) is too light.Resetting to 'minimass' (% f Kg)
			}
			ar_nodes[i].mass = ar_minimass[i];
		}
	}

	m_total_mass = 0;
	for (int i = 0; i < ar_num_nodes; i++)
	{
		if (false) // diagnose truck mass
		{
			// logging node mass 
		}
		m_total_mass += ar_nodes[i].mass;
	}
	// "TOTAL VEHICLE MASS: " + TOSTRING((int)m_total_mass) +" kg"
}

void Actor::calcNodeConnectivityGraph()
{
	int i;

	ar_node_to_node_connections.resize(ar_num_nodes, std::vector<int>());
	ar_node_to_beam_connections.resize(ar_num_nodes, std::vector<int>());

	for (i = 0; i < ar_num_beams; i++)
	{
		if (ar_beams[i].p1 != NULL && ar_beams[i].p2 != NULL &&
			ar_beams[i].p1->pos >= 0 && ar_beams[i].p2->pos >= 0)
		{
			ar_node_to_node_connections[ar_beams[i].p1->pos].push_back(ar_beams[i].p2->pos);
			ar_node_to_beam_connections[ar_beams[i].p1->pos].push_back(i);
			
			ar_node_to_node_connections[ar_beams[i].p2->pos].push_back(ar_beams[i].p1->pos);
			ar_node_to_beam_connections[ar_beams[i].p2->pos].push_back(i);
		}
	}
}

void Actor::calculateAveragePosition()
{
	
	// TODO: using camera positions (from plugin)

	// the classic approach: average over all nodes and beams
	glm::vec3 aposition = glm::vec3(0);
	for (int n = 0; n < ar_num_nodes; n++)
	{
		aposition += ar_nodes[n].AbsPosition;
	}
	m_avg_node_position = aposition / (float)ar_num_nodes;
}

bool SoftBodyLib::Actor::Intersects(Actor* actor, glm::vec3 offset)
{
	glm::vec3 bb_min = ar_bounding_box.getMinimum() + offset;
	glm::vec3 bb_max = ar_bounding_box.getMaximum() + offset;

	AxisAlignedBox bb = AxisAlignedBox(bb_min, bb_max);

	if (!bb.intersects(actor->ar_bounding_box))
		return false;

	// Test own (contactable) beams against others cabs
	for (int i = 0; i < ar_num_beams; i++)
	{
		if (!(ar_beams[i].p1->nd_contacter || ar_beams[i].p1->nd_contactable) ||
			!(ar_beams[i].p2->nd_contacter || ar_beams[i].p2->nd_contactable))
			continue;

		glm::vec3 origin = ar_beams[i].p1->AbsPosition + offset;
		glm::vec3 target = ar_beams[i].p2->AbsPosition + offset;

		Ray ray(origin, target - origin);

		for (int j = 0; j < actor->ar_num_collcabs; j++)
		{
			int index = actor->ar_collcabs[j] * 3;
			glm::vec3 a = actor->ar_nodes[actor->ar_collcabs[index + 0]].AbsPosition;
			glm::vec3 b = actor->ar_nodes[actor->ar_collcabs[index + 1]].AbsPosition;
			glm::vec3 c = actor->ar_nodes[actor->ar_collcabs[index + 2]].AbsPosition;

			auto result = Math::intersects(ray, a, b, c);
			if (result.first && result.second < 1.0f)
			{
				return true;
			}
		}
	}

	// Test own cabs against others (contactable) beams
	for (int i = 0; i < actor->ar_num_beams; i++)
	{
		if (!(actor->ar_beams[i].p1->nd_contacter || actor->ar_beams[i].p1->nd_contactable) ||
			!(actor->ar_beams[i].p2->nd_contacter || actor->ar_beams[i].p2->nd_contactable))
			continue;

		glm::vec3 origin = actor->ar_beams[i].p1->AbsPosition + offset;
		glm::vec3 target = actor->ar_beams[i].p2->AbsPosition + offset;

		Ray ray(origin, target - origin);

		for (int j = 0; j < ar_num_collcabs; j++)
		{
			int index = ar_collcabs[j] * 3;
			glm::vec3 a = ar_nodes[actor->ar_collcabs[index + 0]].AbsPosition;
			glm::vec3 b = ar_nodes[actor->ar_collcabs[index + 1]].AbsPosition;
			glm::vec3 c = ar_nodes[actor->ar_collcabs[index + 2]].AbsPosition;

			auto result = Math::intersects(ray, a, b, c);
			if (result.first && result.second < 1.0f)
			{
				return true;
			}
		}
	}

	return false;
}

glm::vec3 SoftBodyLib::Actor::calculateCollisionOffset(glm::vec3 direction)
{
	if (direction == glm::vec3(0))
		return glm::vec3(0);

	float max_distance = glm::length(direction);
	direction = glm::normalize(direction);

	// collision displacement
	glm::vec3 collision_offset = glm::vec3(0);

	while (glm::length(collision_offset) < max_distance)
	{
		glm::vec3 bb_min = ar_bounding_box.getMinimum() + collision_offset;
		glm::vec3 bb_max = ar_bounding_box.getMaximum() + collision_offset;
		AxisAlignedBox bb = AxisAlignedBox(bb_min, bb_max);

		bool collision = false;

		for (auto actor : m_actor_manager->GetActors())
		{
			if (actor == this)
				continue;
			if (!bb.intersects(actor->ar_bounding_box))
				continue;

			// Test own contactables against other cabs
			if (m_intra_point_col_detector)
			{
				for (int i = 0; i < actor->ar_num_collcabs; i++)
				{
					int tmpv = actor->ar_collcabs[i] * 3;

					node_t* no = &actor->ar_nodes[actor->ar_cabs[tmpv + 0]];
					node_t* na = &actor->ar_nodes[actor->ar_cabs[tmpv + 1]];
					node_t* nb = &actor->ar_nodes[actor->ar_cabs[tmpv + 2]];

					m_intra_point_col_detector->query(no->AbsPosition - collision_offset,
						na->AbsPosition - collision_offset,
						nb->AbsPosition - collision_offset,
						actor->ar_collision_range * 3.0f);

					if (collision = !m_intra_point_col_detector->hit_list.empty())
						break;
				}

				if (collision)
					break;

			}

			float proximity = 0.5f;// std::max(0.5f, std::sqrt(std::max(m_min_camera_radius, actor->m_min_camera_radius)) / 50.f);

			// Test proximity of own nodes against others nodes
			for (int i = 0; i < ar_num_nodes; i++)
			{
				if (!ar_nodes[i].nd_contacter && !ar_nodes[i].nd_contactable)
					continue;

				glm::vec3 query_position = ar_nodes[i].AbsPosition + collision_offset;
				for (int j = 0; j < actor->ar_num_nodes; j++)
				{
					if (!actor->ar_nodes[i].nd_contacter && !actor->ar_nodes[i].nd_contactable)
						continue;

					if (collision = glm::distance2(query_position, actor->ar_nodes[j].AbsPosition) < proximity)
						break;
				}

				if (collision)
					break;
			}

			if (collision)
				break;

		}


		// Test own cabs against others contacters
		if (!collision && m_inter_point_col_detector)
		{
			for (int i = 0; i < ar_num_collcabs; i++)
			{
				int tmpv = ar_collcabs[i] * 3;
				node_t* no = &ar_nodes[ar_cabs[tmpv + 0]];
				node_t* na = &ar_nodes[ar_cabs[tmpv + 1]];
				node_t* nb = &ar_nodes[ar_cabs[tmpv + 2]];

				m_inter_point_col_detector->query(no->AbsPosition + collision_offset,
					na->AbsPosition + collision_offset,
					nb->AbsPosition + collision_offset,
					ar_collision_range * 3.0f);

				if (collision = !m_inter_point_col_detector->hit_list.empty())
					break;
			}
		}

		// Test beams (between contactable nodes) against cabs
		if (!collision)
		{
			for (auto actor : m_actor_manager->GetActors())
			{
				if (actor == this)
					continue;
				if (collision = this->Intersects(actor, collision_offset))
					break;
			}
		}


		if (!collision)
			break;
		
		collision_offset += direction * 0.05f;
	}


	return collision_offset;
}

void SoftBodyLib::Actor::resolveCollisions(glm::vec3 direction)
{
	if (m_intra_point_col_detector)
		m_intra_point_col_detector->UpdateIntraPoint(true);

	if (m_inter_point_col_detector)
		m_inter_point_col_detector->UpdateInterPoint(true);

	glm::vec3 offset = calculateCollisionOffset(direction);

	if (offset == glm::vec3(0))
		return;

	offset += 0.2f * glm::normalize(glm::vec3(offset.x, 0.0f, offset.z));

	resetPosition(ar_nodes[0].AbsPosition.x + offset.x, ar_nodes[0].AbsPosition.z + offset.z, false, getMinHeight() + offset.y);
}

void SoftBodyLib::Actor::resolveCollisions(float max_distance, bool consider_up)
{
	if (m_intra_point_col_detector)
		m_intra_point_col_detector->UpdateIntraPoint(true);

	if (m_inter_point_col_detector)
		m_inter_point_col_detector->UpdateInterPoint(true);

	glm::vec3 u = glm::vec3(0, 1, 0);
	glm::vec3 f = glm::normalize(glm::vec3(getDirection().x, 0.0f, getDirection().z));
	glm::vec3 l = glm::cross(u, f);

	// Calculate an ideal collision avoidance direction (prefer left over right over [front / back / up])
	glm::vec3 left = calculateCollisionOffset(+l * max_distance);
	glm::vec3 right = calculateCollisionOffset(-l * glm::length(left));
	glm::vec3 lateral = glm::length(left) < glm::length(right) * 1.1f ? left : right;

	glm::vec3 front = calculateCollisionOffset(+f * glm::length(lateral));
	glm::vec3 back = calculateCollisionOffset(-f * glm::length(front));
	glm::vec3 sagittal = glm::length(front) < glm::length(back) * 1.1f ? front : back;

	glm::vec3 offset = glm::length(lateral) < glm::length(sagittal) * 1.2f ? lateral : sagittal;

	if (consider_up)
	{
		glm::vec3 up = calculateCollisionOffset(+u * glm::length(offset));
		if (glm::length(up) * 1.2f < glm::length(offset))
			offset = up;
	}

	if (offset == glm::vec3(0))
		return;

	// Additional 20 cm safe-guard (horizontally)
	offset += 0.2f * glm::normalize(glm::vec3(offset.x, 0.0f, offset.z));

	resetPosition(ar_nodes[0].AbsPosition.x + offset.x, ar_nodes[0].AbsPosition.z + offset.z, true, this->getMinHeight() + offset.y);
}

inline void PadBoundingBox(AxisAlignedBox& box) // Internal helper
{
	box.setMaximum(box.getMinimum() - BOUNDING_BOX_PADDING);
	box.setMaximum(box.getMaximum() + BOUNDING_BOX_PADDING);
}

void Actor::UpdateBoundingBoxes()
{
	// Reset
	ar_bounding_box = AxisAlignedBox::BOX_NULL;
	ar_predicted_bounding_box = AxisAlignedBox::BOX_NULL;
	for (size_t i = 0; i < ar_collision_bounding_boxes.size(); ++i)
	{
		ar_collision_bounding_boxes[i] = AxisAlignedBox::BOX_NULL;
		ar_predicted_coll_bounding_boxes[i] = AxisAlignedBox::BOX_NULL;
	}

	// Update
	for (int i = 0; i < ar_num_nodes; i++)
	{
		glm::vec3 vel = ar_nodes[i].Velocity;
		glm::vec3 pos = ar_nodes[i].AbsPosition;
		int16_t cid = ar_nodes[i].nd_coll_bbox_id;

		ar_bounding_box.merge(pos);
		ar_predicted_bounding_box.merge(pos);
		ar_predicted_bounding_box.merge(pos + vel);
		if (cid != node_t::INVALID_BBOX)
		{
			ar_collision_bounding_boxes[cid].merge(pos);
			ar_predicted_coll_bounding_boxes[cid].merge(pos);
			ar_predicted_coll_bounding_boxes[cid].merge(pos + vel);
		}
	}

	// Finalize - add padding
	PadBoundingBox(ar_bounding_box);
	PadBoundingBox(ar_predicted_bounding_box);
	for (size_t i = 0; i < ar_collision_bounding_boxes.size(); ++i)
	{
		PadBoundingBox(ar_collision_bounding_boxes[i]);
		PadBoundingBox(ar_predicted_coll_bounding_boxes[i]);
	}
}

void SoftBodyLib::Actor::UpdatePhysicsOrigin()
{
	if (glm::length2(ar_nodes[0].RelPosition) > 10000.0)
	{
		glm::vec3 offset = ar_nodes[0].RelPosition;
		ar_origin += offset;
		for (int i = 0; i < ar_num_nodes; i++)
		{
			ar_nodes[i].RelPosition -= offset;
		}
	}
}

void Actor::resetPosition(float px, float pz, bool setInitPosition, float miny)
{
	// horizontal displacement
	glm::vec3 offset = glm::vec3(px, ar_nodes[0].AbsPosition.y, pz) - ar_nodes[0].AbsPosition;
	for (int i = 0; i < ar_num_nodes; i++)
	{
		ar_nodes[i].AbsPosition += offset;
		ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
	}

	// vertical displacement
	float vertical_offset = miny - this->getMinHeight();

	// TODO: if has water

	for (int i = 1; i < ar_num_nodes; i++)
	{
		if (ar_nodes[i].nd_no_ground_contact)
			continue;
		float terrainHeight = m_terrain->GetHeightAt(ar_nodes[i].AbsPosition.x, ar_nodes[i].AbsPosition.z);
		vertical_offset += std::max(0.0f, terrainHeight - (ar_nodes[i].AbsPosition.y + vertical_offset));
	}
	for (int i = 0; i < ar_num_nodes; i++)
	{
		ar_nodes[i].AbsPosition.y += vertical_offset;
		ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
	}

	// mesh displacement
	float mesh_offset = 0.0f;
	for (int i = 0; i < ar_num_nodes; i++)
	{
		if (mesh_offset >= 1.0f)
			break;

		if (ar_nodes[i].nd_no_ground_contact)
			continue;

		float offset = mesh_offset;

		while (offset < 1.0f)
		{
			glm::vec3 query = ar_nodes[i].AbsPosition + glm::vec3(0.0f, offset, 0.0f);
			if (m_terrain->GetCollisions()->collisionCorrect(&query, false))
			{
				mesh_offset = offset;
				break;
			}
			offset += 0.001f;
		}
	}
	for (int i = 0; i < ar_num_nodes; i++)
	{
		ar_nodes[i].AbsPosition.y += mesh_offset;
		ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
	}

	resetPosition(glm::vec3(0), setInitPosition);
}

void Actor::resetPosition(glm::vec3 translation, bool setInitPosition)
{
	// total displacement
	if (translation != glm::vec3(0))
	{
		glm::vec3 offset = translation - ar_nodes[0].AbsPosition;
		for (int i = 0; i < ar_num_nodes; i++)
		{
			ar_nodes[i].AbsPosition += offset;
			ar_nodes[i].RelPosition = ar_nodes[i].AbsPosition - ar_origin;
		}
	}

	if (setInitPosition)
	{
		for (int i = 0; i < ar_num_nodes; i++)
		{
			ar_initial_node_positions[i] = ar_nodes[i].AbsPosition;
		}
	}

	this->UpdateBoundingBoxes();
	calculateAveragePosition();
}

float Actor::getMinHeight(bool skip_virtual_nodes)
{
	float height = std::numeric_limits<float>::max();
	for (int i = 0; i < ar_num_nodes; i++)
	{
		if (!skip_virtual_nodes || !ar_nodes[i].nd_no_ground_contact)
		{
			height = std::min(ar_nodes[i].AbsPosition.y, height);
		}
	}
	return (!skip_virtual_nodes || height < std::numeric_limits<float>::max()) ? height : getMinHeight();
}

float Actor::getMaxHeight(bool skip_virtual_nodes)
{
	float height = std::numeric_limits<float>::min();
	for (int i = 0; i < ar_num_nodes; i++)
	{
		if (!skip_virtual_nodes || !ar_nodes[i].nd_no_ground_contact)
		{
			height = std::max(height, ar_nodes[i].AbsPosition.y);
		}
	}
	return (!skip_virtual_nodes || height > std::numeric_limits<float>::min()) ? height : getMaxHeight();
}

//Returns the number of active (non bounded) beams connected to a node
int Actor::GetNumActiveConnectedBeams(int nodeid)
{
	int totallivebeams = 0;
	for (unsigned int ni = 0; ni < ar_node_to_beam_connections[nodeid].size(); ++ni)
	{
		if (!ar_beams[ar_node_to_beam_connections[nodeid][ni]].bm_disabled && !ar_beams[ar_node_to_beam_connections[nodeid][ni]].bounded)
		{
			totallivebeams++;
		}
	}
	return totallivebeams;
}

void Actor::calculateLocalGForces()
{
	glm::vec3 cam_dir = glm::vec3(0); // todo: camera dir
	glm::vec3 cam_roll = glm::vec3(0); // todo camera roll;
	glm::vec3 cam_up = glm::cross(cam_dir, cam_roll);

	glm::vec3 gravity = m_terrain->getGravity();

	// todo
}

bool Actor::CalcForcesEulerPrepare(bool doUpdate)
{
	if (m_ongoing_reset)
		return false;
	if (ar_physics_paused)
		return false;
	if (ar_state != ActorState::LOCAL_SIMULATED)
		return false;

	//if (doUpdate)
	// todo: hook toggle

	// calc hooks
	// calc ropes

	return true;
}

void SoftBodyLib::Actor::CalcForcesEulerCompute(bool doUpdate, int num_steps)
{
	// todo: determine order by order mapping

	this->CalcNodes(); // must be done directly after the inter truck collisions are handled

	this->CalcBeams(doUpdate);
	this->CalcCabCollisions();
}

void SoftBodyLib::Actor::CalcNodes()
{
	const glm::vec3 gravity = m_terrain->getGravity();
	
	for (int i = 0; i < ar_num_nodes; i++)
	{
		// COLLISION
		if (!ar_nodes[i].nd_no_ground_contact)
		{
			glm::vec3 oripos = ar_nodes[i].AbsPosition;
			bool contacted = m_terrain->GetCollisions()->groundCollision(&ar_nodes[i], PHYSICS_DT);
			contacted |= m_terrain->GetCollisions()->nodeCollision(&ar_nodes[i], PHYSICS_DT, false);
			ar_nodes[i].nd_has_ground_contact = contacted;
			if (ar_nodes[i].nd_has_ground_contact || ar_nodes[i].nd_has_mesh_contact) // todo: investigate nd_has_mesh_contact
			{
				ar_last_fuzzy_ground_model = ar_nodes[i].nd_last_collision_gm;

				// FROM ORIGINAL REPOSITORY:
				// Reverts: commit/d11a88142f737528638bd357c38d717c85cebba6#diff-4003254e55aec2c60d21228f375f2a2dL1153
				// Fixes: Gavril Omega Six sliding on ground on the simple2 spawn
				// ar_nodes[i].AbsPosition - oripos is always zero ... dark floating point magic
				ar_nodes[i].RelPosition += ar_nodes[i].AbsPosition - oripos;
			}
		}

		// integration
		if (!ar_nodes[i].nd_immovable)
		{
			ar_nodes[i].Velocity += ar_nodes[i].Forces / ar_nodes[i].mass * PHYSICS_DT;
			ar_nodes[i].RelPosition += ar_nodes[i].Velocity * PHYSICS_DT;
			ar_nodes[i].AbsPosition = ar_origin;
			ar_nodes[i].AbsPosition += ar_nodes[i].RelPosition;
		}

		// prepare next loop (optimisation)
		// we start forces from zero
		// start with gravity
		ar_nodes[i].Forces = glm::vec3(ar_nodes[i].mass * gravity.x, ar_nodes[i].mass * gravity.y, ar_nodes[i].mass * gravity.z);

		float approx_speed = approx_sqrt(glm::length2(ar_nodes[i].Velocity));

		// anti-explosion guard (mach 20)
		if (approx_speed > 6860 && !m_ongoing_reset)
		{
			// todo: reset actor;
			m_ongoing_reset = true;
		}

		// todo: fuse drag override

		if (!ar_disable_aerodyn_turbulent_drag)
		{
			// add viscous drag (turbulent model)
			float defdragxspeed = DEFAULT_DRAG * approx_speed;
			glm::vec3 drag = -defdragxspeed * ar_nodes->Velocity;
			// plus: turbulences
			float maxtur = defdragxspeed * approx_speed * 0.005f;
			drag += maxtur * glm::vec3(frand_11(), frand_11(), frand_11());
			ar_nodes[i].Forces += drag;
		}

		// todo: water

		this->UpdateBoundingBoxes();
	}
}

void SoftBodyLib::Actor::CalcBeams(bool trigger_hooks)
{
	for (int i = 0; i < ar_num_beams; i++)
	{
		if (!ar_beams[i].bm_disabled && !ar_beams[i].bm_inter_actor)
		{
			// Calculate beam length
			glm::vec3 dis = ar_beams[i].p1->RelPosition - ar_beams[i].p2->RelPosition;

			Real dislen = glm::length2(dis);
			Real inverted_dislen = fast_invSqrt(dislen);

			dislen *= inverted_dislen; // todo: what is this doing?

			// Calculate beam's deviation from normal
			Real difftoBeamL = dislen - ar_beams[i].L;

			Real k = ar_beams[i].k;
			Real d = ar_beams[i].d;

			// Calculate beam's rate of change
			float v = glm::dot(ar_beams[i].p1->Velocity - ar_beams[i].p2->Velocity, dis) * inverted_dislen;

			// bounded SHOCK1
			// bounded TRIGGER
			// bounded SHOCK2
			// bounded SHOCK3
			// bounded SUPPORTBEAM
			// bounded SUPPORTBEAM
			// bounded ROPE


			// bm_type == BEAM_HYDRO

			float slen = -k * difftoBeamL - d * v;
			ar_beams[i].stress = slen;

			// Fast test for deformation
			float len = std::abs(slen);
			if (len > ar_beams[i].minmaxposnegstress)
			{
				// todo: not SHOCK1
				// riggid beam?
				if (ar_beams[i].bm_type == BEAM_NORMAL && k != 0.0f)
				{
					// Actual deformation tests
					if (slen > ar_beams[i].maxposstress && difftoBeamL < 0.0f) // compression
					{
						Real yield_length = ar_beams[i].maxposstress / k;
						Real deform = difftoBeamL + yield_length * (1.0f - ar_beams[i].plastic_coef);
						Real Lold = ar_beams[i].L;
						ar_beams[i].L += deform;
						ar_beams[i].L = std::max(MIN_BEAM_LENGTH, ar_beams[i].L);
						slen = slen - (slen - ar_beams[i].maxposstress) * 0.5f;
						len = slen;
						if (ar_beams[i].L > 0.0f && Lold > ar_beams[i].L)
						{
							ar_beams[i].maxposstress *= Lold / ar_beams[i].L;
							ar_beams[i].minmaxposnegstress = std::min(ar_beams[i].maxposstress, -ar_beams[i].maxnegstress);
							ar_beams[i].minmaxposnegstress = std::min(ar_beams[i].minmaxposnegstress, ar_beams[i].strength);
						}

						// For the compression case we do not remove any of the beam's
						// strength for structure stability reasons
						//ar_beams[i].strength += deform * k * 0.5f;
						
						// debug
					}
					else if (slen < ar_beams[i].maxnegstress && difftoBeamL > 0.0f) // expansion
					{
						Real yield_length = ar_beams[i].maxnegstress / k;
						Real deform = difftoBeamL + yield_length * (1.0f - ar_beams[i].plastic_coef);
						Real Lold = ar_beams[i].L;
						ar_beams[i].L += deform;
						slen = slen - (slen - ar_beams[i].maxnegstress) * 0.5f;
						len = -slen;
						if (Lold > 0.0f && ar_beams[i].L > Lold)
						{
							ar_beams[i].maxnegstress *= ar_beams[i].L / Lold;
							ar_beams[i].minmaxposnegstress = std::min(ar_beams[i].maxposstress, -ar_beams[i].maxnegstress);
							ar_beams[i].minmaxposnegstress = std::min(ar_beams[i].minmaxposnegstress, ar_beams[i].strength);
						}
						ar_beams[i].strength -= deform * k;
						
						//debug
					}
				}

				// Test if the beam should break
				if (len > ar_beams[i].strength)
				{
					// sound effect

					//Break the beam only when it is not connected to a node
					//which is a part of a collision triangle and has 2 "live" beams or less
					//connected to it.
					if (!(
							(ar_beams[i].p1->nd_cab_node && GetNumActiveConnectedBeams(ar_beams[i].p1->pos) < 3) ||
							(ar_beams[i].p2->nd_cab_node && GetNumActiveConnectedBeams(ar_beams[i].p2->pos) < 3)
						))
					{
						slen = 0.0f;
						ar_beams[i].bm_broken = true;
						ar_beams[i].bm_broken = true;

						// debug

						// detachergroup check: beam[i] is already broken, check detacher group# == 0/default skip the check ( performance bypass for beams with default setting )
						// only perform this check if this is a master detacher beams (positive detacher group id > 0)
						if (ar_beams[i].detacher_group > 0)
						{
							if (ar_beams[i].detacher_group > 0)
							{
								// cycle once through the other beams
								for (int j = 0; j < ar_num_beams; j++)
								{
									// beam[i] detacher group# == checked beams detacher group# -> delete & disable checked beam
									// do this with all master(positive id) and minor(negative id) beams of this detacher group
									if (abs(ar_beams[j].detacher_group) == ar_beams[i].detacher_group)
									{
										ar_beams[j].bm_broken = true;
										ar_beams[j].bm_disabled = true;
										// debug
									}
								}

								// cycle once through all wheeldetachers
							}
						}

					}
					else
					{
						ar_beams[i].strength = 2.0f * ar_beams[i].minmaxposnegstress;
					}

					// something broke, check buoyant hull


				}

			}

			// At last update the beam forces
			glm::vec3 f = dis;
			f *= (slen * inverted_dislen);
			ar_beams[i].p1->Forces += f;
			ar_beams[i].p2->Forces -= f;

		}
	}
}

void Actor::CalcBeamsInterActor()
{
	for (int i = 0; i < static_cast<int>(ar_inter_beams.size()); i++)
	{
		if (ar_inter_beams[i]->bm_disabled && 
			ar_inter_beams[i]->bm_inter_actor)
		{
			glm::vec3 dis = ar_inter_beams[i]->p1->AbsPosition - ar_inter_beams[i]->p2->AbsPosition;

			Real dislen = glm::length2(dis);
			Real inverted_dislen = fast_invSqrt(dislen);

			dislen *= inverted_dislen;

			// Calculate beam's deviation from normal
			Real difftoBeamL = dislen - ar_inter_beams[i]->L;

			Real k = ar_inter_beams[i]->k;
			Real d = ar_inter_beams[i]->d;

			// ROPE

			// Calculate beam's rate of change
			glm::vec3 v = ar_inter_beams[i]->p1->Velocity - ar_inter_beams[i]->p2->Velocity;

			float slen = -k * (difftoBeamL)-d * glm::dot(v, dis) * inverted_dislen;
			ar_inter_beams[i]->stress = slen;

			// Fast test for deformation
			float len = std::abs(slen);
			if (len > ar_inter_beams[i]->minmaxposnegstress)
			{
				// todo: check not shock
				if (ar_inter_beams[i]->bm_type == BEAM_NORMAL && k != 0.0f)
				{
					// Actual deformation tests
					if (slen > ar_inter_beams[i]->maxposstress && difftoBeamL < 0.0f)
					{
						Real yield_length = ar_inter_beams[i]->maxposstress / k;
						Real deform = difftoBeamL + yield_length * (1.0f - ar_inter_beams[i]->plastic_coef);
						Real Lold = ar_inter_beams[i]->L;
						ar_inter_beams[i]->L += deform;
						ar_inter_beams[i]->L = std::max(MIN_BEAM_LENGTH, ar_inter_beams[i]->L);
						slen = slen - (slen - ar_inter_beams[i]->maxposstress) * 0.5f;
						len = slen;
						if (ar_inter_beams[i]->L > 0.0f && Lold > ar_inter_beams[i]->L)
						{
							ar_inter_beams[i]->maxposstress *= Lold / ar_inter_beams[i]->L;
							ar_inter_beams[i]->minmaxposnegstress = std::min(ar_inter_beams[i]->maxposstress, -ar_inter_beams[i]->maxnegstress);
							ar_inter_beams[i]->minmaxposnegstress = std::min(ar_inter_beams[i]->minmaxposnegstress, ar_inter_beams[i]->strength);
						}

						// For the compression case we do not remove any of the beam's
                        // strength for structure stability reasons
						//ar_inter_beams[i]->strength += deform * k * 0.5f;

						//debug
					}
					else if (slen < ar_inter_beams[i]->maxnegstress && difftoBeamL > 0.0f)
					{
						Real yield_length = ar_inter_beams[i]->maxnegstress / k;
						Real deform = difftoBeamL + yield_length * (1.0f - ar_inter_beams[i]->plastic_coef);
						Real Lold = ar_inter_beams[i]->L;
						ar_inter_beams[i]->L += deform;
						slen = slen - (slen - ar_inter_beams[i]->maxnegstress) * 0.5f;
						len = -slen;
						if (Lold > 0.0f && ar_inter_beams[i]->L > Lold)
						{
							ar_inter_beams[i]->maxposstress *= Lold / ar_inter_beams[i]->L;
							ar_inter_beams[i]->minmaxposnegstress = std::min(ar_inter_beams[i]->maxposstress, -ar_inter_beams[i]->maxnegstress);
							ar_inter_beams[i]->minmaxposnegstress = std::min(ar_inter_beams[i]->minmaxposnegstress, ar_inter_beams[i]->strength);
						}
						ar_inter_beams[i]->strength -= deform * k;

						//debug
					}
				}

				// Test if the beam should break
				if (len > ar_inter_beams[i]->strength)
				{
					// sound effect event

					//Break the beam only when it is not connected to a node
					//which is a part of a collision triangle and has 2 "live" beams or less
					//connected to it.
					if (!(
						(ar_inter_beams[i]->p1->nd_cab_node && GetNumActiveConnectedBeams(ar_inter_beams[i]->p1->pos) < 3) ||
						(ar_inter_beams[i]->p2->nd_cab_node && GetNumActiveConnectedBeams(ar_inter_beams[i]->p2->pos) < 3)
						))
					{
						slen = 0.0f;
						ar_inter_beams[i]->bm_broken = true;
						ar_inter_beams[i]->bm_disabled = true;

						// debug
					}
					else
					{
						ar_inter_beams[i]->strength = 2.0f * ar_inter_beams[i]->minmaxposnegstress;
					}
				}
			}

			// At last update the beam forces
			glm::vec3 f = dis;
			f *= (slen * inverted_dislen);
			ar_inter_beams[i]->p1->Forces += f;
			ar_inter_beams[i]->p2->Forces -= f;
		}
	}
}

void Actor::CalcCabCollisions()
{
	for (int i = 0; i < ar_num_nodes; i++)
	{
		ar_nodes[i].nd_has_mesh_contact = false;
	}
	if (m_intra_point_col_detector != nullptr)
	{
		m_intra_point_col_detector->UpdateIntraPoint();
		ResolveIntraActorCollisions(PHYSICS_DT,
			*m_intra_point_col_detector,
			ar_num_collcabs,
			ar_collcabs,
			ar_cabs,
			ar_intra_collcabrate,
			ar_nodes,
			ar_collision_range,
			*ar_submesh_ground_model);
	}
}
