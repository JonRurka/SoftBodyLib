#include "Actor.h"

using namespace SoftBodyLib;

Actor::Actor(
	int actor_id
	, unsigned int vector_index
) :   ar_instance_id(actor_id)
	, ar_vector_index(vector_index) {

}

Actor::~Actor() {

}

