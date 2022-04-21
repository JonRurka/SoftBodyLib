#pragma once

#include "../Physics.h"
#include "C_Variables.h"
#include "Logger.h"

#ifdef ZERO_MEM_ALLOC
#include "../../utils/ZeroedMemoryAllocator.h"
#endif

namespace SoftBodyLib {

	class PointColDetector
#ifdef ZERO_MEM_ALLOC
		: public ZeroedMemoryAllocator
#endif
	{
	public:
		struct pointid_t
		{
			Actor* actor;
			short node_id;
		};

		std::vector<pointid_t*> hit_list;

		PointColDetector(Actor* actor) : m_actor(actor), m_object_list_size(-1), m_bbmin(0), m_bbmax(0){};

		
		void UpdateIntraPoint(bool contactables = false);

		
		void UpdateInterPoint(bool ignorestate = false);


		void query(const glm::vec3& vec1, const glm::vec3& vec2, const glm::vec3& vec3, const float enlargeBB);

	private:

		struct refelem_t
		{
			pointid_t* pidref;
			const float* point;
		};

		struct kdnode_t
		{
			float min;
			float max;

			int begin;
			float middle;
			int end;

			refelem_t* ref;
		};


		Actor* m_actor;
		std::vector<Actor*>		m_linked_actors;
		std::vector<Actor*>		m_collision_partners;
		std::vector<refelem_t>	m_ref_list;
		std::vector<pointid_t>	m_pointid_list;
		std::vector<kdnode_t>	m_kdtree;
		glm::vec3				m_bbmin;
		glm::vec3				m_bbmax;
		int						m_object_list_size;

		void queryrec(int kdindex, int axis);
		void build_kdtree_incr(int axis, int index);
		void partintwo(const int start, const int median, const int end, const int axis, float& minex, float& maxex);
		void update_structures_for_contacters(bool ignoreinternal);
	};

}

