#include "Actor.h"

using namespace SoftBodyLib;

static const glm::vec3 BOUNDING_BOX_PADDING(0.05f, 0.05f, 0.05f);

Actor::Actor(
	int actor_id
	, unsigned int vector_index
	, std::shared_ptr<File> def
	, ActorSpawnRequest rq
) :   ar_instance_id(actor_id)
	, ar_vector_index(vector_index) {

}

Actor::~Actor() {

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

