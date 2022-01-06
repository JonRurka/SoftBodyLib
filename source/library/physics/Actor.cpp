#include "Actor.h"

using namespace SoftBodyLib;

static const glm::vec3 BOUNDING_BOX_PADDING(0.05f, 0.05f, 0.05f);

Actor::Actor(
	int actor_id
	, unsigned int vector_index
	, std::shared_ptr<File> def
	, ActorSpawnRequest rq
	, TerrainManager_Base* ter
) :   ar_instance_id(actor_id)
	, ar_vector_index(vector_index) 
	, m_avg_node_position_prev(rq.asr_position)
	, m_avg_node_position(rq.asr_position)
	, terrain(ter)

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
		float terrainHeight = terrain->GetHeightAt(ar_nodes[i].AbsPosition.x, ar_nodes[i].AbsPosition.z);
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
			if (terrain->GetCollisions()->collisionCorrect(&query, false))
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


