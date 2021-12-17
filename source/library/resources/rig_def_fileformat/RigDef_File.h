#pragma once

#include "rigdef.h"
//#include "physics/SimData.h"
#include "physics/SimConstants.h"

/* -------------------------------------------------------------------------- */
/* Directive SET_NODE_DEFAULTS                                                */
/* -------------------------------------------------------------------------- */

struct NodeDefaults
{
    NodeDefaults();

    float load_weight;
    float friction;
    float volume;
    float surface;
    unsigned int options; //!< Bit flags
};

/* -------------------------------------------------------------------------- */
/* Directive SET_BEAM_DEFAULTS_SCALE                                          */
/* -------------------------------------------------------------------------- */

struct BeamDefaultsScale
{
    BeamDefaultsScale() :
        springiness(1),
        damping_constant(1),
        deformation_threshold_constant(1),
        breaking_threshold_constant(1)
    {}

    float springiness;
    float damping_constant;
    float deformation_threshold_constant;
    float breaking_threshold_constant;
};


/* -------------------------------------------------------------------------- */
/* Directive SET_BEAM_DEFAULTS                                                */
/* -------------------------------------------------------------------------- */

struct BeamDefaults
{
    BeamDefaults() : // NOTE: -1.f is 'empty value'; replaced by constant in parser.
        springiness(-1.f),
        damping_constant(-1.f),
        deformation_threshold(-1.f),
        visual_beam_diameter(-1.f),
        beam_material_name("tracks/beam"),
        plastic_deform_coef(0.f), // This is a default
        breaking_threshold(-1.f),
        enable_advanced_deformation(false),
        is_plastic_deform_coef_user_defined(false),
        is_user_defined(false)
    {}

    float GetScaledSpringiness()
    {
        return springiness * scale.springiness;
    }

    float GetScaledDamping()
    {
        return damping_constant * scale.damping_constant;
    }

    float GetScaledBreakingThreshold()
    {
        return breaking_threshold * scale.breaking_threshold_constant;
    }

    inline float GetScaledDeformThreshold() const
    {
        return deformation_threshold * scale.deformation_threshold_constant;
    }

    float springiness;
    float damping_constant;
    float deformation_threshold;
    float breaking_threshold;
    float visual_beam_diameter;
    std::string beam_material_name;
    float plastic_deform_coef;
    bool enable_advanced_deformation; //!< Informs whether "enable_advanced_deformation" directive preceded these defaults.
    bool is_plastic_deform_coef_user_defined;
    bool is_user_defined; //!< Informs whether these data were read from "set_beam_defaults" directive or filled in by the parser on startup.
    BeamDefaultsScale scale;
};

/* -------------------------------------------------------------------------- */
/* Hybrid section MINIMASS                                                    */
/* -------------------------------------------------------------------------- */

struct MinimassPreset
{
    enum Option
    {
        OPTION_n_FILLER = 'n',     //!< Updates the global minimass
        OPTION_l_SKIP_LOADED = 'l'  //!< Only apply minimum mass to nodes without "L" option.
    };

    MinimassPreset() : min_mass(DEFAULT_MINIMASS)
    {}

    explicit MinimassPreset(float m) : min_mass(m)
    {}

    float min_mass; //!< minimum node mass in Kg
};

/* -------------------------------------------------------------------------- */
/* Section GLOBALS                                                            */
/* -------------------------------------------------------------------------- */

struct Globals
{
    Globals() :
        dry_mass(0), /* The default */
        cargo_mass(0) /* The default */
    {}

    float dry_mass;
    float cargo_mass;
    std::string material_name;
};


/* -------------------------------------------------------------------------- */
/* Section BEAMS                                                              */
/* -------------------------------------------------------------------------- */

struct Beam
{
    Beam() :
        options(0),
        extension_break_limit(0), /* This is default */
        has_extension_break_limit(false),
        detacher_group(0) /* 0 = Default detacher group */
    {}

    Node::Ref nodes[2];
    unsigned int options; //!< Bit flags
    float extension_break_limit;
    bool has_extension_break_limit;
    int detacher_group;
    std::shared_ptr<BeamDefaults> defaults;
};

/* -------------------------------------------------------------------------- */
/* Section COLLISIONBOXES                                                     */
/* -------------------------------------------------------------------------- */

struct CollisionBox
{
    CollisionBox()
    {
        nodes.reserve(25);
    }

    std::vector<Node::Ref> nodes;
};

/* -------------------------------------------------------------------------- */
/* Section FLEXBODIES                                                         */
/* -------------------------------------------------------------------------- */

struct Flexbody
{
    Flexbody() :
        offset(glm::vec3(0)),
        rotation(glm::vec3(0))
    {
    }

    Node::Ref reference_node;
    Node::Ref x_axis_node;
    Node::Ref y_axis_node;
    glm::vec3 offset;
    glm::vec3 rotation;
    std::string mesh_name;
    //std::list<Animation> animations;
    std::vector<Node::Range> node_list_to_import; //!< Node ranges are disallowed in fileformatversion >=450
    std::vector<Node::Ref> node_list;
    //CameraSettings camera_settings;
};

/* -------------------------------------------------------------------------- */
/* Section NODECOLLISION                                                      */
/* -------------------------------------------------------------------------- */

struct NodeCollision
{
    NodeCollision() :
        radius(0)
    {}

    Node::Ref node;
    float radius;
};

/* -------------------------------------------------------------------------- */
/* Section SUBMESH                                                            */
/* -------------------------------------------------------------------------- */

struct Cab
{
    Cab() :
        options(0)
    {}



    Node::Ref nodes[3];
    unsigned int options;
};

struct Texcoord
{
    Texcoord() :
        u(0),
        v(0)
    {}

    Node::Ref node;
    float u;
    float v;
};

struct Submesh
{
    Submesh() :
        backmesh(false)
    {}

    bool backmesh;
    std::vector<Texcoord> texcoords;
    std::vector<Cab> cab_triangles;
};

struct File
{
//public:
    /** Modular part of vehicle (part of file wrapped in 'section ~ end_section' tags)
    */
    class Module
    {
    public:
        Module(std::string const& name);

        std::string name;

        std::vector<unsigned int>          contacter_nodes;

        std::vector<Beam>                  beams;
        std::vector<CollisionBox>          collision_boxes;
        std::vector<Node::Ref>             contacters;
        std::vector<std::shared_ptr<Flexbody>>	flexbodies;
        std::shared_ptr<Globals>           globals;
        std::vector<Node>                  nodes;
        std::vector<NodeCollision>         node_collisions;
        std::string                        submeshes_ground_model_name;
        std::vector<Submesh>               submeshes;
    };


    enum Keyword
    {
        KEYWORD_ADD_ANIMATION = 1,
        //KEYWORD_ADVDRAG, // not supported yet
        KEYWORD_AIRBRAKES,
        KEYWORD_ANIMATORS,
        KEYWORD_ANTI_LOCK_BRAKES,
        KEYWORD_AXLES,
        KEYWORD_AUTHOR,
        KEYWORD_BACKMESH,
        KEYWORD_BEAMS,
        KEYWORD_BRAKES,
        KEYWORD_CAB,
        KEYWORD_CAMERARAIL,
        KEYWORD_CAMERAS,
        KEYWORD_CINECAM,
        KEYWORD_COLLISIONBOXES,
        KEYWORD_COMMANDS,
        KEYWORD_COMMANDS2,
        KEYWORD_CONTACTERS,
        KEYWORD_CRUISECONTROL,
        KEYWORD_DESCRIPTION,
        KEYWORD_DETACHER_GROUP,
        KEYWORD_DISABLEDEFAULTSOUNDS,
        KEYWORD_ENABLE_ADVANCED_DEFORM,
        KEYWORD_END,
        KEYWORD_END_SECTION,
        KEYWORD_ENGINE,
        KEYWORD_ENGOPTION,
        KEYWORD_ENGTURBO,
        KEYWORD_ENVMAP,
        KEYWORD_EXHAUSTS,
        KEYWORD_EXTCAMERA,
        KEYWORD_FILEFORMATVERSION,
        KEYWORD_FILEINFO,
        KEYWORD_FIXES,
        KEYWORD_FLARES,
        KEYWORD_FLARES2,
        KEYWORD_FLEXBODIES,
        KEYWORD_FLEXBODY_CAMERA_MODE,
        KEYWORD_FLEXBODYWHEELS,
        KEYWORD_FORWARDCOMMANDS,
        KEYWORD_FUSEDRAG,
        KEYWORD_GLOBALS,
        KEYWORD_GUID,
        KEYWORD_GUISETTINGS,
        KEYWORD_HELP,
        KEYWORD_HIDE_IN_CHOOSER,
        KEYWORD_HOOKGROUP, // obsolete, ignored
        KEYWORD_HOOKS,
        KEYWORD_HYDROS,
        KEYWORD_IMPORTCOMMANDS,
        KEYWORD_INTERAXLES,
        KEYWORD_LOCKGROUPS,
        KEYWORD_LOCKGROUP_DEFAULT_NOLOCK,
        KEYWORD_MANAGEDMATERIALS,
        KEYWORD_MATERIALFLAREBINDINGS,
        KEYWORD_MESHWHEELS,
        KEYWORD_MESHWHEELS2,
        KEYWORD_MINIMASS,
        KEYWORD_NODECOLLISION,
        KEYWORD_NODES,
        KEYWORD_NODES2,
        KEYWORD_PARTICLES,
        KEYWORD_PISTONPROPS,
        KEYWORD_PROP_CAMERA_MODE,
        KEYWORD_PROPS,
        KEYWORD_RAILGROUPS,
        KEYWORD_RESCUER,
        KEYWORD_RIGIDIFIERS,
        KEYWORD_ROLLON,
        KEYWORD_ROPABLES,
        KEYWORD_ROPES,
        KEYWORD_ROTATORS,
        KEYWORD_ROTATORS2,
        KEYWORD_SCREWPROPS,
        KEYWORD_SECTION,
        KEYWORD_SECTIONCONFIG,
        KEYWORD_SET_BEAM_DEFAULTS,
        KEYWORD_SET_BEAM_DEFAULTS_SCALE,
        KEYWORD_SET_COLLISION_RANGE,
        KEYWORD_SET_DEFAULT_MINIMASS,
        KEYWORD_SET_INERTIA_DEFAULTS,
        KEYWORD_SET_MANAGEDMATS_OPTIONS,
        KEYWORD_SET_NODE_DEFAULTS,
        KEYWORD_SET_SHADOWS,
        KEYWORD_SET_SKELETON_SETTINGS,
        KEYWORD_SHOCKS,
        KEYWORD_SHOCKS2,
        KEYWORD_SHOCKS3,
        KEYWORD_SLIDENODE_CONNECT_INSTANT,
        KEYWORD_SLIDENODES,
        KEYWORD_SLOPE_BRAKE,
        KEYWORD_SOUNDSOURCES,
        KEYWORD_SOUNDSOURCES2,
        //KEYWORD_SOUNDSOURCES3, // not supported yet
        KEYWORD_SPEEDLIMITER,
        KEYWORD_SUBMESH,
        KEYWORD_SUBMESH_GROUNDMODEL,
        KEYWORD_TEXCOORDS,
        KEYWORD_TIES,
        KEYWORD_TORQUECURVE,
        KEYWORD_TRACTION_CONTROL,
        KEYWORD_TRANSFER_CASE,
        KEYWORD_TRIGGERS,
        KEYWORD_TURBOJETS,
        KEYWORD_TURBOPROPS,
        KEYWORD_TURBOPROPS2,
        KEYWORD_VIDEOCAMERA,
        KEYWORD_WHEELDETACHERS,
        KEYWORD_WHEELS,
        KEYWORD_WHEELS2,
        KEYWORD_WINGS,

        KEYWORD_INVALID = 0xFFFFFFFF
    };

    enum Section
    {
        SECTION_AIRBRAKES,
        SECTION_AUTHOR,
        SECTION_ANIMATORS,
        SECTION_ANTI_LOCK_BRAKES,
        SECTION_AXLES,
        SECTION_BEAMS,
        SECTION_BRAKES,
        SECTION_CAMERAS,
        SECTION_CAMERA_RAIL,
        SECTION_CINECAM,
        SECTION_COLLISION_BOXES,
        SECTION_COMMANDS,
        SECTION_COMMANDS_2,
        SECTION_CONTACTERS,
        SECTION_ENGINE,
        SECTION_ENGOPTION,
        SECTION_ENGTURBO,
        SECTION_EXHAUSTS,
        SECTION_FIXES,
        SECTION_FLARES,
        SECTION_FLARES_2,
        SECTION_FLEXBODIES,
        SECTION_FLEX_BODY_WHEELS,
        SECTION_FUSEDRAG,
        SECTION_GLOBALS,
        SECTION_GUI_SETTINGS,
        SECTION_HELP,
        SECTION_HOOKS,
        SECTION_HYDROS,
        SECTION_INTERAXLES,
        SECTION_LOCKGROUPS,
        SECTION_MANAGED_MATERIALS,
        SECTION_MAT_FLARE_BINDINGS,
        SECTION_MESH_WHEELS,
        SECTION_MESH_WHEELS_2,
        SECTION_MINIMASS,
        SECTION_NODES,
        SECTION_NODES_2,
        SECTION_NODE_COLLISION,
        SECTION_PARTICLES,
        SECTION_PISTONPROPS,
        SECTION_PROPS,
        SECTION_RAILGROUPS,
        SECTION_ROPABLES,
        SECTION_ROPES,
        SECTION_ROTATORS,
        SECTION_ROTATORS_2,
        SECTION_SCREWPROPS,
        SECTION_SHOCKS,
        SECTION_SHOCKS_2,
        SECTION_SHOCKS_3,
        SECTION_SLIDENODES,
        SECTION_SOUNDSOURCES,
        SECTION_SOUNDSOURCES2,
        SECTION_SUBMESH,
        SECTION_SLOPE_BRAKE,
        SECTION_TIES,
        SECTION_TORQUE_CURVE,
        SECTION_TRACTION_CONTROL,
        SECTION_TRANSFER_CASE,
        SECTION_TRIGGERS,
        SECTION_TRUCK_NAME, //!< The very start of file	
        SECTION_TURBOJETS,
        SECTION_TURBOPROPS,
        SECTION_TURBOPROPS_2,
        SECTION_VIDEO_CAMERA,
        SECTION_WHEELDETACHERS,
        SECTION_WHEELS,
        SECTION_WHEELS_2,
        SECTION_WINGS,

        SECTION_NONE,       //!< Right after rig name, for example.

        SECTION_INVALID = 0xFFFFFFFF
    };

    enum Subsection
    {
        SUBSECTION_NONE = 0,

        SUBSECTION__FLEXBODIES__PROPLIKE_LINE,
        SUBSECTION__FLEXBODIES__FORSET_LINE,

        SUBSECTION__SUBMESH__TEXCOORDS,
        SUBSECTION__SUBMESH__CAB,

        SUBSECTION_INVALID = 0xFFFFFFFF
    };

    static const char* SubsectionToString(Subsection subsection);

    static const char* SectionToString(Section section);

    static const char* KeywordToString(Keyword keyword);

    File();

    unsigned int file_format_version;
    bool enable_advanced_deformation;
    std::string name;
    float collision_range;

    // File hash
    std::string hash;

    // Vehicle modules(caled 'sections' in truckfile doc)
    std::shared_ptr<Module> root_module; //!< Required to exist. `shared_ptr` is used for unified handling with other modules.
    std::map< std::string, std::shared_ptr<Module> > user_modules;



};