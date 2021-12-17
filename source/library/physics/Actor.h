#pragma once

#include "Physics.h"


#ifdef ZERO_MEM_ALLOC
#include "../utils/ZeroedMemoryAllocator.h"
#endif

namespace SoftBodyLib {

    using namespace Util;

    /// Softbody object; can be anything from soda can to a space shuttle
    /// Former name: `Beam` (that's why scripting uses `BeamClass`)
    class Actor
#ifdef ZERO_MEM_ALLOC
        : public ZeroedMemoryAllocator
#endif
    {

    public:


        Actor(
            int actor_id
            , unsigned int vector_index
            //, std::shared_ptr<RigDef::File> def
            //, ActorSpawnRequest rq
        );

        ~Actor();


        std::vector<Actor*> getAllLinkedActors() { return m_linked_actors; }; //!< Returns a list of all connected (hooked) actors


        node_t*              ar_nodes;
        int                  ar_num_nodes;

        beam_t*              ar_beams;
        int                  ar_num_beams;

        std::vector<beam_t*> ar_inter_beams;       //!< Beams connecting 2 actors


        AxisAlignedBox      ar_bounding_box;     //!< standard bounding box (surrounds all nodes of an actor)

        std::vector<std::vector<int>>  ar_node_to_node_connections;


        int               ar_num_contactable_nodes; //!< Total number of nodes which can contact ground or cabs
        int               ar_num_contacters; //!< Total number of nodes which can selfcontact cabs


        int               ar_collcabs[MAX_CABS];
        collcab_rate_t    ar_inter_collcabrate[MAX_CABS];
        collcab_rate_t    ar_intra_collcabrate[MAX_CABS];
        int               ar_num_collcabs;


        int               ar_instance_id;              //!< Static attr; session-unique ID
        unsigned int      ar_vector_index;             //!< Sim attr; actor element index in std::vector<m_actors>


        // Gameplay state
        ActorState        ar_state;

        // Bit flags
        bool ar_update_physics : 1; //!< Physics state; Should this actor be updated (locally) in the next physics step?

        bool ar_collision_relevant : 1;      //!< Physics state;

    private:

        std::vector<Actor*>  m_linked_actors;           //!< Sim state; other actors linked using 'hooks'
    };

}