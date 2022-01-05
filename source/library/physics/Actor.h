#pragma once

#include "Physics.h"
#include "resources/resources.h"


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
        friend class ActorSpawner;
        friend class ActorManager;

    public:


        Actor(
            int actor_id
            , unsigned int vector_index
            , std::shared_ptr<File> def
            , ActorSpawnRequest rq
        );

        ~Actor();


        //! @{ Physic state functions
        void              resetPosition(glm::vec3 translation, bool setInitPosition); //!< Moves the actor to given world coords.
        void              resetPosition(float px, float pz, bool setInitPosition, float miny); //!< Moves the actor to given world coords.

        float             getRotation();
        glm::vec3         getDirection();
        glm::vec3         getPosition();


        std::vector<Actor*> getAllLinkedActors() { return m_linked_actors; }; //!< Returns a list of all connected (hooked) actors



        void              UpdateBoundingBoxes();
        void              calculateAveragePosition();



        node_t*              ar_nodes;
        int                  ar_num_nodes;

        beam_t*              ar_beams;
        int                  ar_num_beams;

        std::vector<beam_t*> ar_inter_beams;       //!< Beams connecting 2 actors


        AxisAlignedBox      ar_bounding_box;     //!< standard bounding box (surrounds all nodes of an actor)
        AxisAlignedBox      ar_predicted_bounding_box;

        std::vector<std::vector<int>>  ar_node_to_node_connections;
        std::vector<AxisAlignedBox>  ar_collision_bounding_boxes; //!< smart bounding boxes, used for determining the state of an actor (every box surrounds only a subset of nodes)
        std::vector<AxisAlignedBox>  ar_predicted_coll_bounding_boxes;

        float                     ar_initial_total_mass;

        std::vector<float>        ar_initial_node_masses;
        std::vector<glm::vec3>     ar_initial_node_positions;
        std::vector<std::pair<float, float>> ar_initial_beam_defaults;


        int               ar_num_contactable_nodes; //!< Total number of nodes which can contact ground or cabs
        int               ar_num_contacters; //!< Total number of nodes which can selfcontact cabs

        int               ar_cabs[MAX_CABS * 3];
        int               ar_num_cabs;

        int               ar_collcabs[MAX_CABS];
        collcab_rate_t    ar_inter_collcabrate[MAX_CABS];
        collcab_rate_t    ar_intra_collcabrate[MAX_CABS];
        int               ar_num_collcabs;

        std::vector<float>             ar_minimass; //!< minimum node mass in Kg


        int               ar_instance_id;              //!< Static attr; session-unique ID
        unsigned int      ar_vector_index;             //!< Sim attr; actor element index in std::vector<m_actors>

        glm::vec3         ar_origin;                   //!< Physics state; base position for softbody nodes
        ground_model_t*   ar_submesh_ground_model;

        float             ar_collision_range;             //!< Physics attr

        // Gameplay state
        ActorState        ar_state;

        // Bit flags
        bool ar_update_physics : 1; //!< Physics state; Should this actor be updated (locally) in the next physics step?

        bool ar_collision_relevant : 1;      //!< Physics state;

    private:

        void              RecalculateNodeMasses(Real total);
        void              calcNodeConnectivityGraph();

        std::shared_ptr<File>      m_definition;

        float             m_spawn_rotation;

        std::vector<Actor*>  m_linked_actors;           //!< Sim state; other actors linked using 'hooks'

        PointColDetector* m_inter_point_col_detector;   //!< Physics
        PointColDetector* m_intra_point_col_detector;   //!< Physics


        float             m_total_mass;            //!< Physics state; total mass in Kg

        float             m_load_mass;             //!< Physics attr; predefined load mass in Kg
        int               m_masscount;             //!< Physics attr; Number of nodes loaded with l option
        float             m_dry_mass;              //!< Physics attr;
    };

}