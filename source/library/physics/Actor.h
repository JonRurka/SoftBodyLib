#pragma once

#include "Physics.h"


#ifdef ZERO_MEM_ALLOC
#include "../utils/ZeroedMemoryAllocator.h"
#endif

namespace SoftBodyLib {

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


        node_t* ar_nodes;
        int                  ar_num_nodes;
        beam_t* ar_beams;
        int                  ar_num_beams;
        std::vector<beam_t*> ar_inter_beams;       //!< Beams connecting 2 actors


        std::vector<std::vector<int>>  ar_node_to_node_connections;


        int               ar_instance_id;              //!< Static attr; session-unique ID
        unsigned int      ar_vector_index;             //!< Sim attr; actor element index in std::vector<m_actors>


        // Gameplay state
        ActorState        ar_state;

        
    };

}