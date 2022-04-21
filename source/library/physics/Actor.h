#pragma once

#include "Physics.h"
#include "resources/resources.h"
#include "terrain/terrain.h"
#include "C_Variables.h"


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
            , ActorManager* mngr
            , TerrainManager_Base* ter
        );

        ~Actor();


        //! @{ Physic state functions
        void              resetPosition(glm::vec3 translation, bool setInitPosition); //!< Moves the actor to given world coords.
        void              resetPosition(float px, float pz, bool setInitPosition, float miny); //!< Moves the actor to given world coords.

        float             getRotation();
        glm::vec3         getDirection();
        glm::vec3         getPosition();

        float             getMinHeight(bool skip_virtual_nodes = true);
        float             getMaxHeight(bool skip_virtual_nodes = true);


        std::vector<Actor*> getAllLinkedActors() { return m_linked_actors; }; //!< Returns a list of all connected (hooked) actors



        void              UpdateBoundingBoxes();


        void              calculateLocalGForces();             //!< Derive the truck local g-forces from the global ones

        /// Virtually moves the actor at most 'direction.length()' meters towards 'direction' trying to resolve any collisions
        /// Returns a minimal offset by which the actor needs to be moved to resolve any collisions
        //  Both PointColDetectors need to be updated accordingly before calling th
        glm::vec3 calculateCollisionOffset(glm::vec3 direction);

        bool              Intersects(Actor* actor, glm::vec3 offset = glm::vec3(0));  //!< Slow intersection test
        /// Moves the actor at most 'direction.length()' meters towards 'direction' to resolve any collisions
        void              resolveCollisions(glm::vec3 direction);

        /// Auto detects an ideal collision avoidance direction (front, back, left, right, up)
        /// Then moves the actor at most 'max_distance' meters towards that direction to resolve any collisions
        void              resolveCollisions(float max_distance, bool consider_up);
        int               GetNumActiveConnectedBeams(int nodeid);     //!< Returns the number of active (non bounded) beams connected to a node
        void              calculateAveragePosition();
        void              UpdatePhysicsOrigin();

        node_t*              ar_nodes{ nullptr };
        int                  ar_num_nodes{ 0 };

        beam_t*              ar_beams{ nullptr };
        int                  ar_num_beams{ 0 };

        std::vector<beam_t*> ar_inter_beams;       //!< Beams connecting 2 actors


        AxisAlignedBox      ar_bounding_box;     //!< standard bounding box (surrounds all nodes of an actor)
        AxisAlignedBox      ar_predicted_bounding_box;

        std::vector<std::vector<int>>  ar_node_to_node_connections;
        std::vector<std::vector<int>>  ar_node_to_beam_connections;
        std::vector<AxisAlignedBox>  ar_collision_bounding_boxes; //!< smart bounding boxes, used for determining the state of an actor (every box surrounds only a subset of nodes)
        std::vector<AxisAlignedBox>  ar_predicted_coll_bounding_boxes;

        float                      ar_initial_total_mass{ 0 };

        std::vector<float>         ar_initial_node_masses;
        std::vector<glm::vec3>     ar_initial_node_positions;
        std::vector<std::pair<float, float>> ar_initial_beam_defaults;


        int               ar_num_contactable_nodes{ 0 }; //!< Total number of nodes which can contact ground or cabs
        int               ar_num_contacters{ 0 }; //!< Total number of nodes which can selfcontact cabs

        int               ar_cabs[MAX_CABS * 3];
        int               ar_num_cabs{ 0 };

        int               ar_collcabs[MAX_CABS];
        collcab_rate_t    ar_inter_collcabrate[MAX_CABS];
        collcab_rate_t    ar_intra_collcabrate[MAX_CABS];
        int               ar_num_collcabs{ 0 };

        std::vector<float>             ar_minimass; //!< minimum node mass in Kg


        int               ar_instance_id{ 0 };              //!< Static attr; session-unique ID
        unsigned int      ar_vector_index{ 0 };             //!< Sim attr; actor element index in std::vector<m_actors>

        glm::vec3         ar_origin;                   //!< Physics state; base position for softbody nodes
        ground_model_t*   ar_submesh_ground_model{ nullptr };


        ActorType         ar_driveable;                //!< Sim attr; marks vehicle type and features
        
        float             ar_collision_range{ 0 };             //!< Physics attr
        float             ar_top_speed{ 0 };                   //!< Sim state
        ground_model_t* ar_last_fuzzy_ground_model;

        PointColDetector* m_inter_point_col_detector{ nullptr };   //!< Physics
        PointColDetector* m_intra_point_col_detector{ nullptr };   //!< Physics


        // Gameplay state
        ActorState        ar_state;

        // Bit flags
        bool ar_update_physics : 1; //!< Physics state; Should this actor be updated (locally) in the next physics step?
        bool ar_disable_aerodyn_turbulent_drag : 1; //!< Physics state
        bool ar_collision_relevant : 1;      //!< Physics state;
        bool ar_physics_paused : 1;   //!< Sim state
    private:

        bool              CalcForcesEulerPrepare(bool doUpdate);
        void              CalcForcesEulerCompute(bool doUpdate, int num_steps);
        
        void              CalcNodes();
        void              CalcBeams(bool trigger_hooks);
        void              CalcBeamsInterActor();
        void              CalcCabCollisions();
        
        void              RecalculateNodeMasses(Real total);
        void              calcNodeConnectivityGraph();

        std::shared_ptr<File>      m_definition;

        float             m_spawn_rotation;

        std::vector<Actor*>  m_linked_actors;           //!< Sim state; other actors linked using 'hooks'

        glm::vec3     m_avg_node_position;          //!< average node position
        glm::vec3     m_avg_node_position_prev;
        glm::vec3     m_avg_node_velocity;

        
        ActorManager* m_actor_manager{ nullptr };
        TerrainManager_Base* m_terrain{ nullptr };


        float             m_total_mass;            //!< Physics state; total mass in Kg

        float             m_load_mass;             //!< Physics attr; predefined load mass in Kg
        int               m_masscount;             //!< Physics attr; Number of nodes loaded with l option
        float             m_dry_mass;              //!< Physics attr;

        bool              m_ongoing_reset;         //!< Hack to prevent position/rotation creep during interactive truck reset
    };

}


