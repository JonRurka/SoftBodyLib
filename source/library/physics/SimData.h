#pragma once

#include "../ForwardDeclarations.h"

namespace SoftBodyLib {

    enum CollisionEventFilter : short
    {
        EVENT_NONE = 0,
        EVENT_ALL,
        EVENT_AVATAR,
        EVENT_TRUCK,
        EVENT_AIRPLANE,
        EVENT_BOAT,
        EVENT_DELETE
    };

    enum BeamType : short
    {
        BEAM_NORMAL,
        BEAM_HYDRO,
        BEAM_VIRTUAL,         //!< Excluded from mass calculations, visuals permanently disabled
    };

    enum SpecialBeam : short
    {
        NOSHOCK,        //!< not a shock
        SHOCK1,         //!< shock1
        SHOCK2,         //!< shock2
        SHOCK3,         //!< shock3
        TRIGGER,        //!< trigger
        SUPPORTBEAM,    //!<
        ROPE            //!<
    };

    // --------------------------------
    // Soft body physics

    /// Physics: A vertex in the softbody structure
    struct node_t
    {
        // REFACTOR IN PROGRESS: Currently nodes are adressed mostly by pointers or int32_t indices,
        //     although there was always a hidden soft limit of 2^16 nodes (because of `short node_t::pos`).
        //     Let's use `uint16_t` indices everywhere to be clear.      ~ only_a_ptr, 04/2018
        static const uint16_t INVALID_IDX = std::numeric_limits<uint16_t>::max();
        static const int8_t   INVALID_BBOX = -1;

        node_t() { memset(this, 0, sizeof(node_t)); nd_coll_bbox_id = INVALID_BBOX; }
        node_t(size_t _pos) { memset(this, 0, sizeof(node_t)); nd_coll_bbox_id = INVALID_BBOX; pos = static_cast<short>(_pos); }

        glm::vec3   RelPosition;             //!< relative to the local physics origin (one origin per actor) (shaky)
        glm::vec3   AbsPosition;             //!< absolute position in the world (shaky)
        glm::vec3   Velocity;
        glm::vec3   Forces;

        float      mass;
        float      buoyancy;
        float      friction_coef;
        float      surface_coef;
        float      volume_coef;

        int16_t         pos;                     //!< This node's index in Actor::ar_nodes array.
        int16_t         nd_coll_bbox_id;         //!< Optional attribute (-1 = none) - multiple collision bounding boxes defined in truckfile
        int16_t         nd_lockgroup;            //!< Optional attribute (-1 = default, 9999 = deny lock) - used in the hook lock logic

        // Bit flags
        bool            nd_cab_node : 1;           //!< Attr; This node is part of collision triangle
        bool            nd_rim_node : 1;           //!< Attr; This node is part of a rim
        bool            nd_tyre_node : 1;          //!< Attr; This node is part of a tyre
        bool            nd_contacter : 1;          //!< Attr; User-defined
        bool            nd_contactable : 1;        //!< Attr; This node will be treated as contacter on inter truck collisions
        bool            nd_has_ground_contact : 1; //!< Physics state
        bool            nd_has_mesh_contact : 1;   //!< Physics state
        bool            nd_immovable : 1;          //!< Attr; User-defined
        bool            nd_loaded_mass : 1;        //!< User defined attr; mass is calculated from 'globals/loaded-mass' rather than 'globals/dry-mass'
        bool            nd_no_ground_contact : 1;  //!< User-defined attr; node ignores contact with ground
        bool            nd_override_mass : 1;      //!< User defined attr; mass is user-specified rather than calculated (override the calculation)
        bool            nd_under_water : 1;        //!< State; GFX hint
        bool            nd_no_mouse_grab : 1;      //!< Attr; User-defined

        float      nd_avg_collision_slip;   //!< Physics state; average slip velocity across the last few physics frames
        glm::vec3   nd_last_collision_slip;  //!< Physics state; last collision slip vector
        glm::vec3   nd_last_collision_force; //!< Physics state; last collision force
        ground_model_t* nd_last_collision_gm;    //!< Physics state; last collision 'ground model' (surface definition)
    };

    /// Simulation: An edge in the softbody structure
    struct beam_t
    {
        beam_t() { memset(this, 0, sizeof(beam_t)); }

        node_t* p1;
        node_t* p2;
        float      k;                     //!< tensile spring
        float      d;                     //!< damping factor
        float      L;                     //!< length
        float      minmaxposnegstress;
        float      maxposstress;
        float      maxnegstress;
        float      strength;
        float      stress;
        float      plastic_coef;
        int             detacher_group;        //!< Attribute: detacher group number (integer)
        SpecialBeam     bounded;
        BeamType        bm_type;
        bool            bm_inter_actor;        //!< in case p2 is on another actor
        //Actor* bm_locked_actor;       //!< in case p2 is on another actor
        bool            bm_disabled;
        bool            bm_broken;

        float      shortbound;
        float      longbound;
        float      refL;                  //!< reference length

        shock_t* shock;

        float      initial_beam_strength; //!< for reset
        float      default_beam_deform;   //!< for reset

        float      debug_k;               //!< debug shock spring_rate
        float      debug_d;               //!< debug shock damping
        float      debug_v;               //!< debug shock velocity
    };

    struct shock_t
    {
        shock_t() { memset(this, 0, sizeof(shock_t)); }

        int beamid;
        int flags;

        bool trigger_enabled;       //!< general trigger,switch and blocker state
        float trigger_switch_state; //!< needed to avoid doubleswitch, bool and timer in one
        float trigger_boundary_t;   //!< optional value to tune trigger_switch_state autorelease
        int trigger_cmdlong;        //!< F-key for trigger injection longbound-check
        int trigger_cmdshort;       //!< F-key for trigger injection shortbound-check
        int last_debug_state;       //!< smart debug output

        float springin;  //!< shocks2 & shocks3
        float dampin;    //!< shocks2 & shocks3
        float springout; //!< shocks2 & shocks3
        float dampout;   //!< shocks2 & shocks3

        float sprogin;   //!< shocks2
        float dprogin;   //!< shocks2
        float sprogout;  //!< shocks2
        float dprogout;  //!< shocks2

        float splitin;   //!< shocks3
        float dslowin;   //!< shocks3
        float dfastin;   //!< shocks3
        float splitout;  //!< shocks3
        float dslowout;  //!< shocks3
        float dfastout;  //!< shocks3

        float sbd_spring;           //!< set beam default for spring
        float sbd_damp;             //!< set beam default for damping
    };

    struct collcab_rate_t
    {
        int rate;     // remaining amount of physics cycles to be skipped
        int distance; // distance (in physics cycles) to the previous collision check
    };



    // --------------------------------
    // some non-actor structs

    struct collision_box_t
    {
        bool virt;
        bool refined;
        bool selfrotated;
        bool camforced;
        bool enabled;
        CollisionEventFilter event_filter;
        short eventsourcenum;
        glm::vec3 lo;           //!< absolute collision box
        glm::vec3 hi;           //!< absolute collision box
        glm::vec3 center;       //!< center of rotation
        glm::quat rot;       //!< rotation
        glm::quat unrot;     //!< rotation
        glm::vec3 selfcenter;   //!< center of self rotation
        glm::quat selfrot;   //!< self rotation
        glm::quat selfunrot; //!< self rotation
        glm::vec3 relo;         //!< relative collision box
        glm::vec3 rehi;         //!< relative collision box
        glm::vec3 campos;       //!< camera position
    };

    struct ground_model_t
    {
        float va;                       //!< adhesion velocity
        float ms;                       //!< static friction coefficient
        float mc;                       //!< sliding friction coefficient
        float t2;                       //!< hydrodynamic friction (s/m)
        float vs;                       //!< stribeck velocity (m/s)
        float alpha;                    //!< steady-steady
        float strength;                 //!< ground strength

        float fluid_density;            //!< Density of liquid
        float flow_consistency_index;   //!< general drag coefficient

        //! if flow_behavior_index<1 then liquid is Pseudoplastic (ketchup, whipped cream, paint)
        //! if =1 then liquid is Newtoni'an fluid
        //! if >1 then liquid is Dilatant fluid (less common)
        float flow_behavior_index;


        float solid_ground_level;       //!< how deep the solid ground is
        float drag_anisotropy;          //!< Upwards/Downwards drag anisotropy

        int fx_type;

        //Ogre::ColourValue fx_colour;
        int fx_colour;

        char name[256];
        char basename[256];
        char particle_name[256];

        int fx_particle_amount;         //!< amount of particles

        float fx_particle_min_velo;     //!< minimum velocity to display sparks
        float fx_particle_max_velo;     //!< maximum velocity to display sparks
        float fx_particle_fade;         //!< fade coefficient
        float fx_particle_timedelta;    //!< delta for particle animation
        float fx_particle_velo_factor;  //!< velocity factor
        float fx_particle_ttl;
    };
}