#pragma once

#include "physics/Physics.h"
#include "resources/resources.h"
#include "terrain/terrain.h"
#include "utils/utils.h"
#include "Logger.h"
#include "SimContext.h"


//extern "C"
//{
// EXPORTED int Exe_Test();

// EXPORTED int Init(int);


// TESTS

// EXPORTED void UpdateActors(float m_dt_remainder, float m_simulation_time);

// EXPORTED bool TestTrisCollision(glm::vec3 t_Pos1, glm::vec3 t_Pos2, glm::vec3 t_Pos3, glm::vec3 testP);


//}

// #### Logger

EXPORTED int Logger_PrepareMessages();

EXPORTED int Logger_GetMessage(int index, char* err_source, char* message);


// #### SimContext

EXPORTED void* SimContext_New();

EXPORTED C_Vec3 SimContext_Test(void* sim_context, SoftBodyLib::ActorSpawnRequest rq);

EXPORTED bool SimContext_LoadTerrain(void* sim_context, void* terrain_mgr, void* collisions, float gravity);

EXPORTED void SimContext_UnloadTerrain(void* sim_context);

EXPORTED void* SimContext_SpawnActor(void* sim_context, SoftBodyLib::ActorSpawnRequest rq, void* file_builder);

EXPORTED void  SimContext_DeleteActor(void* sim_context, void* actor);

EXPORTED void SimContext_ModifyActor(void* sim_context);

EXPORTED void SimContext_UpdateActors(void* sim_context, float dt);

EXPORTED void* SimContext_GetActorManager(void* sim_context);

EXPORTED int SimContext_GetSimState(void* sim_context);

EXPORTED bool SimContext_IsSimulationPaused(void* sim_context);

EXPORTED void SimContext_SetSimulationPaused(void* sim_context, bool v);


// #### ActorManager

EXPORTED void ActorManager_SetSimulationSpeed(void* handle, float speed);

EXPORTED float ActorManager_GetSimulationSpeed(void* handle);


// #### Actor

EXPORTED int Actor_getNumNodes(void* handle);

EXPORTED int Actor_GetNodes(void* handle, void** nodes);

EXPORTED int Actor_GetNumBeams(void* handle);

EXPORTED void Actor_GetBeams(void* handle, void** beams);

EXPORTED void* Actor_GetNodeRef(void* handle, int index);

EXPORTED void* Actor_GetBeamRef(void* handle, int index);

EXPORTED void* Actor_GetInter_point_col_detector(void* handle);

EXPORTED void* Actor_GetIntra_point_col_detector(void* handle);

EXPORTED int Actor_GetNum_contactable_nodes(void* handle);

EXPORTED int Actor_GetNum_contacters(void* handle);

EXPORTED void* Actor_GetBounding_Box(void* handle);

EXPORTED void* Actor_GetPredicted_Bounding_Box(void* handle);

// #### node_t

EXPORTED int	  Node_t_getPosition(void* handle);

EXPORTED C_Vec3   Node_t_getRelPosition(void* handle);

EXPORTED C_Vec3   Node_t_getAbsPosition(void* handle);

EXPORTED C_Vec3   Node_t_getVelocity(void* handle);

EXPORTED C_Vec3   Node_t_getForces(void* handle);

EXPORTED C_Vec3   Node_t_getRelPosition_idx(void* ac_handle, int index);

EXPORTED C_Vec3   Node_t_getAbsPosition_idx(void* ac_handle, int index);

EXPORTED C_Vec3   Node_t_getVelocity_idx(void* ac_handle, int index);

EXPORTED C_Vec3   Node_t_getForces_idx(void* ac_handle, int index);



EXPORTED void   Node_t_setRelPosition(void* handle, C_Vec3 pos);

EXPORTED void   Node_t_setAbsPosition(void* handle, C_Vec3 pos);

EXPORTED void   Node_t_setVelocity(void* handle, C_Vec3 vel);

EXPORTED void   Node_t_setForces(void* handle, C_Vec3 force);

EXPORTED void   Node_t_setRelPosition_idx(void* ac_handle, int index, C_Vec3 pos);

EXPORTED void   Node_t_setAbsPosition_idx(void* ac_handle, int index, C_Vec3 pos);

EXPORTED void   Node_t_setVelocity_idx(void* ac_handle, int index, C_Vec3 vel);

EXPORTED void   Node_t_setForces_idx(void* ac_handle, int index, C_Vec3 force);

// #### beam_t

EXPORTED void* Beam_t_getP1(void* handle);

EXPORTED void* Beam_t_getP2(void* handle);


// #### collision_box_t

// #### ground_model_t



// #### Collisions_Base

EXPORTED    float Collisions_Base_getSurfaceHeight(void* handle, float x, float z);

EXPORTED    float Collisions_Base_getSurfaceHeightBelow(void* handle, float x, float z, float height);

//bool Collisions_Base_collisionCorrect(void* handle, glm::vec3* refpos, bool envokeScriptCallbacks = true);

EXPORTED    bool Collisions_Base_groundCollision(void* handle, void* node, float dt);

EXPORTED    bool Collisions_Base_isInside_1(void* handle, C_Vec3 pos, const std::string& inst, const std::string& box, float border = 0);

EXPORTED    bool Collisions_Base_isInside_2(void* handle, C_Vec3 pos, void* cbox, float border = 0);

EXPORTED    bool Collisions_Base_nodeCollision(void* handle, void* node, float dt);

EXPORTED    int Collisions_Base_addCollisionBox(void* handle,
    bool rotating,
    bool virt,
    C_Vec3 pos,
    C_Vec3 rot,
    C_Vec3 l,
    C_Vec3 h,
    C_Vec3 sr,
    const std::string& eventname,
    const std::string& instancename,
    bool forcecam,
    C_Vec3 campos,
    C_Vec3 sc,
    C_Vec3 dr,
    short event_filter,
    int scripthandler);

EXPORTED    int Collisions_Base_addCollisionTri(void* handle, C_Vec3 p1, C_Vec3 p2, C_Vec3 p3, SoftBodyLib::ground_model_t gm);

EXPORTED    void Collisions_Base_removeCollisionBox(void* handle, int number);

EXPORTED    void Collisions_Base_removeCollisionTri(void* handle, int number);

EXPORTED    void Collisions_Base_clearEventCache(void* handle);

EXPORTED    void* Collisions_Base_getCollisionAAB(void* handle);

EXPORTED    C_Vec3 C_primitiveCollision(void* node, C_Vec3 velocity, float mass, C_Vec3 normal, float dt, SoftBodyLib::ground_model_t gm, float penetration);

// #### collisions

EXPORTED void* Collisions_New(float terrn_size_x, float terrn_size_y, float terrn_size_z);

EXPORTED void Collisions_addGroundModel(void* handle, const char* name, SoftBodyLib::ground_model_t model);

EXPORTED void Collisions_setDefaultGroundModels(void* handle);

// #### PointColDetector

EXPORTED void* PointColDetector_New(void* actor);

//EXPORTED int PointColDetector_hit_list(void* handle, void** list, int size);

EXPORTED int PointColDetector_hit_list(void* handle, SoftBodyLib::PointColDetector::pointid_t* list, int size);

EXPORTED int PointColDetector_numHits(void* handle);

EXPORTED void PointColDetector_UpdateIntraPoint(void* handle, bool contactables = false);

EXPORTED void PointColDetector_UpdateInterPoint(void* handle, bool ignorestate = false);

EXPORTED void PointColDetector_query(void* handle, const C_Vec3 vec1, const C_Vec3 vec2, const C_Vec3 vec3, const float enlargeBB);

EXPORTED C_Vec3 PointColDetector_Get_bbmin(void* handle);

EXPORTED C_Vec3 PointColDetector_Get_bbmax(void* handle);

//EXPORTED void* PointColDetector_pointid_t_getActor(void* handle);

//EXPORTED short PointColDetector_pointid_t_getNode_id(void* handle);


// #### TerrainManager_Base

EXPORTED void TerrainManager_Base_getTerrainName(void* handle, char*, int len);

EXPORTED void* TerrainManager_Base_GetCollisions(void* handle);

EXPORTED void TerrainManager_Base_SetCollisions(void* handle, void* col);

EXPORTED void TerrainManager_Base_setGravity(void* handle, C_Vec3 value);

EXPORTED C_Vec3 TerrainManager_Base_getGravity(void* handle);

EXPORTED float TerrainManager_Base_GetHeightAt(void* handle, float x, float z);

EXPORTED C_Vec3 TerrainManager_Base_GetNormalAt(void* handle, float x, float y, float z);

EXPORTED C_Vec3 TerrainManager_Base_getMaxTerrainSize(void* handle);

EXPORTED void* TerrainManager_Base_getTerrainCollisionAAB(void* handle);


// #### Terrain

EXPORTED void* SimpleTerrainManager_New();

EXPORTED void* SimpleTerrainManager_New_col(void* col);

EXPORTED void SimpleTerrainManager_setGroundHeight(void* handle, float height);



// #### FileBuilder

EXPORTED void* FileBuilder_New();

EXPORTED void FileBuilder_SetGlobals(void* handle,
	float dry_mass,
	float cargo_mass,
	const char* material_name);

EXPORTED void FileBuilder_SetNodeDefaults(void* handle,
	float load_weight,
	float friction,
	float volume,
	float surface,
	unsigned int options);

EXPORTED void FileBuilder_SetBeamDefaultsScale(void* handle,
	float springiness,
	float damping_constan,
	float deformation_threshold_constant,
	float breaking_threshold_constant);

EXPORTED void FileBuilder_SetBeamDefaults(void* handle,
	float springiness,
	float damping_constant,
	float deformation_threshold,
	float breaking_threshold,
	float visual_beam_diameter,
	const char* beam_material_name,
	float plastic_deform_coef,
	bool _enable_advanced_deformation, //!< Informs whether "enable_advanced_deformation" directive preceded these defaults.
	bool _is_plastic_deform_coef_user_defined,
	bool _is_user_defined //!< Informs whether these data were read from "set_beam_defaults" directive or filled in by the parser on startup.
);

EXPORTED void FileBuilder_SetMinimassPreset(void* handle, float min_mass);

EXPORTED void FileBuilder_AddNode(void* handle, const char* id, float x, float y, float z, bool loadWeight, float weight);

EXPORTED void FileBuilder_AddBeam(void* handle, const char* node_1, const char* node_2, bool canBreak, float breakLimit);

EXPORTED void FileBuilder_NewSubmesh(void* handle);

EXPORTED void FileBuilder_FlushSubmesh(void* handle);

EXPORTED void FileBuilder_AddCab(void* handle, const char* n1, const char* n2, const char* n3, int option);

EXPORTED void FileBuilder_AddContacter(void* handle, const char* n1);

// #### AxisAlignedBox

EXPORTED C_Vec3 AxisAlignedBox_getMinimum(void* handle);

EXPORTED C_Vec3 AxisAlignedBox_getMaximum(void* handle);

EXPORTED C_Vec3 AxisAlignedBox_getSize(void* handle);

EXPORTED C_Vec3 AxisAlignedBox_getCorner(void* handle, int cornerToGet);

EXPORTED bool AxisAlignedBox_contains(void* handle, glm::vec3 v);