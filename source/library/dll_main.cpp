#include "dll_main.h"

using namespace SoftBodyLib;

#define PHYSICS_DT 0.0005f // fixed dt of 0.5 ms


/*
int Exe_Test() {
	int Init_return_val = Init(7);
	return Init_return_val;
}

int Init(int inpar) {
	printf("Init\n");

	//UpdateActors(0, 0.03);


	glm::vec3 a = glm::vec3(0, 0, 0);
	glm::vec3 b = glm::vec3(0.5, 1, 0);
	glm::vec3 c = glm::vec3(1, 0, 0);

	glm::vec3 t = glm::vec3(0.5, 0.5, 0);

	bool didCollide = TestTrisCollision(a, b, c, t);

	printf("Collision: %s\n", didCollide ? "true" : "false");

	return 0;
}

void UpdateActors(float m_dt_remainder, float m_simulation_time) {
	float m_simulation_speed = 1;
	int m_physics_steps = 0;

	float dt = m_simulation_time;

	// do not allow dt > 1/20
	dt = std::min(dt, 1.0f / 20.0f);
	printf("1 dt: %f \n", dt);

	dt *= m_simulation_speed;
	printf("2 dt: %f \n", dt);

	dt += m_dt_remainder;
	printf("3 dt: %f \n", dt);

	m_physics_steps = dt / PHYSICS_DT;
	printf("4 m_physics_steps: %i \n", m_physics_steps);

	if (m_physics_steps == 0)
	{
		return;
	}

	m_dt_remainder = dt - (m_physics_steps * PHYSICS_DT);
	printf("5 m_dt_remainder: %f \n", m_dt_remainder);

	dt = PHYSICS_DT * m_physics_steps;
	printf("6 dt: %f \n", dt);



}

bool TestTrisCollision(glm::vec3 t_Pos1, glm::vec3 t_Pos2, glm::vec3 t_Pos3, glm::vec3 testP)
{
	const Triangle triangle(t_Pos1, t_Pos2, t_Pos3);

	printf("normal: %f, %f, %f\n", triangle.normal().x, triangle.normal().y, triangle.normal().z);

	const CartesianToTriangleTransform transform(triangle);

	// transform point to triangle local coordinates
	const auto local_point = transform(testP);
	printf("a: %f \n", local_point.barycentric.alpha);
	printf("b: %f \n", local_point.barycentric.beta);
	printf("g: %f \n", local_point.barycentric.gamma);
	printf("distance: %f \n", local_point.distance);

	// collision test
	const bool is_colliding = SoftBodyLib::InsideTriangleTest(local_point, DEFAULT_COLLISION_RANGE);

	return is_colliding;

}
*/

// #### Logger

// C log batch populated via prepare()
std::vector<SoftBodyLib::Logger::LogEntry> log_batch;
int Logger_PrepareMessages()
{
	log_batch = Logger::GetLogEntries();
	return log_batch.size();
}

/*void Logger_GetMessages(SoftBodyLib::Logger::LogEntry* entries)
{
	for (int i = 0; i < log_batch.size(); i++)
	{
		entries[i] = log_batch[i];
	}
}*/

int Logger_GetMessage(int index, char* err_source, char* message)
{
	Logger::LogEntry ent = log_batch[index];
	strcpy(err_source, ent.source.c_str());
	strcpy(message, ent.message.c_str());

	return (int)ent.log_level;
}


// #### SimContext

void* SimContext_New()
{
	return new SimContext();
}

C_Vec3 SimContext_Test(void* sim_context, SoftBodyLib::ActorSpawnRequest rq)
{
	SimContext* cont = (SimContext*)sim_context;

	return cont->DoTest(rq);
}

bool SimContext_LoadTerrain(void* sim_context, void* terrain_mgr, void* collisions, float gravity)
{
	SimContext* cont = (SimContext*)sim_context;

	TerrainManager_Base* terr = (TerrainManager_Base*)terrain_mgr;
	Collisions_Base* col = (Collisions_Base*)collisions;

	TerrainManager_Base::TerrainSettings terrn_settings;
	terrn_settings.terrain_mgr = terr;
	terrn_settings.collisions = col;
	terrn_settings.gravity = gravity;

	return cont->LoadTerrain(terrn_settings);
}

void SimContext_UnloadTerrain(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	cont->UnloadTerrain();
}

void* SimContext_SpawnActor(void* sim_context, SoftBodyLib::ActorSpawnRequest rq, void* file_builder)
{
	SimContext* cont = (SimContext*)sim_context;

	FileBuilder* builder = (FileBuilder*)file_builder;

	return cont->SpawnActor(rq, builder);
}

void SimContext_DeleteActor(void* sim_context, void* actor)
{
	SimContext* cont = (SimContext*)sim_context;

	Actor* act = (Actor*)actor;

	cont->DeleteActor(act);
}

void SimContext_ModifyActor(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	// todo
}

void SimContext_UpdateActors(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	cont->UpdateActors();
}

void* SimContext_GetActorManager(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	return cont->GetActorManager();
}

int SimContext_GetSimState(void* sim_context)
{
	SimContext* cont = (SimContext*)sim_context;

	return (int)cont->GetSimState();
}



// #### Actor

int Actor_getNumNodes(void* handle)
{
	Actor* actor = (Actor*)handle;
	return actor->ar_num_nodes;
}

int Actor_GetNodes(void* handle, void** nodes)
{
	Actor* actor = (Actor*)handle;

	/*for (int i = 0; i < actor->ar_num_nodes; i++)
	{
		nodes[i] = &actor->ar_nodes[i];
	}*/


	//node_t** ptrs = new node_t*[actor->ar_num_nodes];
	for (int i = 0; i < actor->ar_num_nodes; i++)
	{
		void* ptr = &actor->ar_nodes[i];

		printf("%x\n", ptr);

		nodes[i] = &ptr;
	}

	//nodes = (void**)ptrs;

	//delete ptrs;
	//memcpy(nodes, actor->ar_nodes, actor->ar_num_nodes);

	return actor->ar_num_nodes;
}

int Actor_GetNumBeams(void* handle)
{
	Actor* actor = (Actor*)handle;
	return actor->ar_num_beams;
}

void Actor_GetBeams(void* handle, void** beams)
{
	Actor* actor = (Actor*)handle;

	for (int i = 0; i < actor->ar_num_beams; i++)
	{
		beams[i] = &actor->ar_beams[i];
	}
}

void* Actor_GetNodeRef(void* handle, int index)
{
	Actor* actor = (Actor*)handle;
	return &actor->ar_nodes[index];
}

EXPORTED void* Actor_GetBeamRef(void* handle, int index)
{
	Actor* actor = (Actor*)handle;
	return &actor->ar_beams[index];
}

void* Actor_GetInter_point_col_detector(void* handle)
{
	Actor* actor = (Actor*)handle;
	return actor->m_inter_point_col_detector;
}

void* Actor_GetIntra_point_col_detector(void* handle)
{
	Actor* actor = (Actor*)handle;
	return actor->m_intra_point_col_detector;
}

EXPORTED int Actor_GetNum_contactable_nodes(void* handle)
{
	Actor* actor = (Actor*)handle;
	return actor->ar_num_contactable_nodes;
}

EXPORTED int Actor_GetNum_contacters(void* handle)
{
	Actor* actor = (Actor*)handle;
	return actor->ar_num_contacters;
}


// #### node_t

int	  Node_t_getPosition(void* handle)
{
	node_t* node = (node_t*)handle;
	return (int)node->pos;
}

C_Vec3   Node_t_getRelPosition(void* handle)
{
	node_t* node = (node_t*)handle;
	return C_Vec3::To(node->RelPosition);
}

C_Vec3   Node_t_getAbsPosition(void* handle)
{
	node_t* node = (node_t*)handle;
	return C_Vec3::To(node->AbsPosition);
}

C_Vec3   Node_t_getVelocity(void* handle)
{
	node_t* node = (node_t*)handle;
	return C_Vec3::To(node->Velocity);
}

C_Vec3   Node_t_getForces(void* handle)
{
	node_t* node = (node_t*)handle;
	return C_Vec3::To(node->Forces);
}


C_Vec3   Node_t_getRelPosition_idx(void* ac_handle, int index)
{
	Actor* ac = (Actor*)ac_handle;
	return C_Vec3::To(ac->ar_nodes[index].RelPosition);
}

C_Vec3   Node_t_getAbsPosition_idx(void* ac_handle, int index)
{
	Actor* ac = (Actor*)ac_handle;
	return C_Vec3::To(ac->ar_nodes[index].AbsPosition);
}

C_Vec3   Node_t_getVelocity_idx(void* ac_handle, int index)
{
	Actor* ac = (Actor*)ac_handle;
	return C_Vec3::To(ac->ar_nodes[index].Velocity);
}

C_Vec3   Node_t_getForces_idx(void* ac_handle, int index)
{
	Actor* ac = (Actor*)ac_handle;
	return C_Vec3::To(ac->ar_nodes[index].Forces);
}


// #### beam_t

void* Beam_t_getP1(void* handle)
{
	beam_t* beam = (beam_t*)handle;
	return beam->p1;
}

void* Beam_t_getP2(void* handle)
{
	beam_t* beam = (beam_t*)handle;
	return beam->p2;
}

// #### collision_box_t

// #### ground_model_t


// #### Collisions_Base

float Collisions_Base_getSurfaceHeight(void* handle, float x, float z)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->getSurfaceHeight(x, z);
}

float Collisions_Base_getSurfaceHeightBelow(void* handle, float x, float z, float height)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->getSurfaceHeightBelow(x, z, height);
}

bool Collisions_Base_groundCollision(void* handle, void* node, float dt)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->groundCollision((SoftBodyLib::node_t*)node, dt);
}

bool Collisions_Base_isInside_1(void* handle, C_Vec3 pos, const std::string& inst, const std::string& box, float border)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->isInside(C_Vec3::From(pos), inst, box, border);
}

bool Collisions_Base_isInside_2(void* handle, C_Vec3 pos, void* cbox, float border)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->isInside(C_Vec3::From(pos), (SoftBodyLib::collision_box_t*)cbox, border);
}

bool Collisions_Base_nodeCollision(void* handle, void* node, float dt)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->nodeCollision((SoftBodyLib::node_t*)node, dt, false);
}

int Collisions_Base_addCollisionBox(void* handle,
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
	int scripthandler)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->addCollisionBox(
		rotating,
		virt,
		C_Vec3::From(pos),
		C_Vec3::From(rot),
		C_Vec3::From(l),
		C_Vec3::From(h),
		C_Vec3::From(sr),
		eventname,
		instancename,
		forcecam,
		C_Vec3::From(campos),
		C_Vec3::From(sc),
		C_Vec3::From(dr),
		(CollisionEventFilter)event_filter,
		scripthandler);
}

int Collisions_Base_addCollisionTri(void* handle,
	C_Vec3 p1,
	C_Vec3 p2,
	C_Vec3 p3,
	void* gm)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return col->addCollisionTri(C_Vec3::From(p1), C_Vec3::From(p2), C_Vec3::From(p2), (ground_model_t*)gm);
}

void Collisions_Base_removeCollisionBox(void* handle, int number)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	col->removeCollisionBox(number);
}

void Collisions_Base_removeCollisionTri(void* handle, int number)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	col->removeCollisionTri(number);
}

void Collisions_Base_clearEventCache(void* handle)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	col->clearEventCache();
}

void* Collisions_Base_getCollisionAAB(void* handle)
{
	Collisions_Base* col = (Collisions_Base*)handle;

	return &col->getCollisionAAB();
}

C_Vec3 C_primitiveCollision(void* node, C_Vec3 velocity, float mass, C_Vec3 normal, float dt, void* gm, float penetration)
{
	return C_Vec3::To(primitiveCollision((node_t*)node, C_Vec3::From(velocity), mass, C_Vec3::From(normal), dt, (ground_model_t*)gm, penetration));
}


// #### Collisions

void* Collisions_New(float terrn_size_x, float terrn_size_y, float terrn_size_z)
{
	return new Collisions(glm::vec3(terrn_size_x, terrn_size_y, terrn_size_z));
}


// #### PointColDetector

void* PointColDetector_New(void* actor)
{
	return new PointColDetector((Actor*)actor);
}

/*int PointColDetector_hit_list(void* handle, void** list, int size)
{
	PointColDetector* p_col = (PointColDetector*)handle;

	int i = 0;
	for (auto hit : p_col->hit_list)
	{
		if (i >= size)
			break;

		list[i] = &hit;
		i++;
	}

	return i;
}*/

int PointColDetector_hit_list(void* handle, PointColDetector::pointid_t* list, int size)
{
	PointColDetector* p_col = (PointColDetector*)handle;

	int i = 0;
	for (auto hit : p_col->hit_list)
	{
		if (i >= size)
			break;

		list[i] = *hit;
		i++;
	}

	return i;
}

void PointColDetector_UpdateIntraPoint(void* handle, bool contactables)
{
	PointColDetector* p_col = (PointColDetector*)handle;

	p_col->UpdateIntraPoint(contactables);
}

void PointColDetector_UpdateInterPoint(void* handle, bool ignorestate)
{
	PointColDetector* p_col = (PointColDetector*)handle;

	p_col->UpdateInterPoint(ignorestate);
}

void PointColDetector_query(void* handle, const C_Vec3 vec1, const C_Vec3 vec2, const C_Vec3 vec3, const float enlargeBB)
{
	PointColDetector* p_col = (PointColDetector*)handle;

	p_col->query(C_Vec3::From(vec1), C_Vec3::From(vec2), C_Vec3::From(vec3), enlargeBB);
}

/*EXPORTED void* PointColDetector_pointid_t_getActor(void* handle)
{
	PointColDetector::pointid_t* p_col = (PointColDetector::pointid_t*)handle;

	return p_col->actor;
}

EXPORTED short PointColDetector_pointid_t_getNode_id(void* handle)
{
	PointColDetector::pointid_t* p_col = (PointColDetector::pointid_t*)handle;

	return p_col->node_id;
}*/


// #### TerrainManager_Base

void TerrainManager_Base_getTerrainName(void* handle, char* dest, int len)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	std::string res = terr->getTerrainName();

	strcpy(dest, res.c_str());
}

void* TerrainManager_Base_GetCollisions(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->GetCollisions();
}

void TerrainManager_Base_SetCollisions(void* handle, void* col)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	terr->SetCollisions((Collisions_Base*)col);
}

void TerrainManager_Base_setGravity(void* handle, C_Vec3 value)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;
	terr->setGravity(C_Vec3::From(value));
}

C_Vec3 TerrainManager_Base_getGravity(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return C_Vec3::To(terr->getGravity());
}

float TerrainManager_Base_GetHeightAt(void* handle, float x, float z)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return terr->GetHeightAt(x, z);
}

C_Vec3 TerrainManager_Base_GetNormalAt(void* handle, float x, float y, float z)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return C_Vec3::To(terr->GetNormalAt(x, y, z));
}

C_Vec3 TerrainManager_Base_getMaxTerrainSize(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return C_Vec3::To(terr->getMaxTerrainSize());
}

void* TerrainManager_Base_getTerrainCollisionAAB(void* handle)
{
	TerrainManager_Base* terr = (TerrainManager_Base*)handle;

	return &terr->getTerrainCollisionAAB();
}


// #### SimpleTerrainManager

void* SimpleTerrainManager_New()
{
	return new SimpleTerrainManager();
}

void* SimpleTerrainManager_New_col(void* col)
{
	return new SimpleTerrainManager((Collisions_Base*)col);
}

void SimpleTerrainManager_setGroundHeight(void* handle, float height)
{
	SimpleTerrainManager* terr = (SimpleTerrainManager*)handle;
	terr->setGroundHeight(height);
}

// #### FileBuilder

EXPORTED void* FileBuilder_New()
{
	FileBuilder* builder = new FileBuilder();
	return builder;
}

void FileBuilder_SetGlobals(void* handle,
	float dry_mass,
	float cargo_mass,
	const char* material_name)
{
	FileBuilder* builder = (FileBuilder*)handle;

	builder->SetGlobals(dry_mass, cargo_mass, material_name);
}

void FileBuilder_SetNodeDefaults(void* handle,
	float load_weight,
	float friction,
	float volume,
	float surface,
	unsigned int options)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->SetNodeDefaults(load_weight, friction, volume, surface, options);
}

void FileBuilder_SetBeamDefaultsScale(void* handle,
	float springiness,
	float damping_constan,
	float deformation_threshold_constant,
	float breaking_threshold_constant)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->SetBeamDefaultsScale(springiness, damping_constan, deformation_threshold_constant, breaking_threshold_constant);
}

void FileBuilder_SetBeamDefaults(void* handle,
	float springiness,
	float damping_constant,
	float deformation_threshold,
	float breaking_threshold,
	float visual_beam_diameter,
	const char* beam_material_name,
	float plastic_deform_coef,
	bool enable_advanced_deformation, //!< Informs whether "enable_advanced_deformation" directive preceded these defaults.
	bool is_plastic_deform_coef_user_defined,
	bool is_user_defined //!< Informs whether these data were read from "set_beam_defaults" directive or filled in by the parser on startup.
)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->SetBeamDefaults(
		springiness,
		damping_constant,
		deformation_threshold,
		breaking_threshold,
		visual_beam_diameter,
		std::string(beam_material_name),
		plastic_deform_coef,
		enable_advanced_deformation,
		is_plastic_deform_coef_user_defined,
		is_user_defined
	);
}

void FileBuilder_SetMinimassPreset(void* handle, float min_mass)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->SetMinimassPreset(min_mass);
}

// TODO: Module support
void FileBuilder_AddNode(void* handle, const char* id, float x, float y, float z, bool loadWeight, float weight)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->AddNode(id, x, y, z, loadWeight, weight);
}

void FileBuilder_AddBeam(void* handle, const char* node_1, const char* node_2, bool canBreak, float breakLimit)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->AddBeam(node_1, node_2, canBreak, breakLimit);
}

void FileBuilder_NewSubmesh(void* handle)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->NewSubmesh();
}

void FileBuilder_FlushSubmesh(void* handle)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->FlushSubmesh();
}

void FileBuilder_AddCab(void* handle, const char* n1, const char* n2, const char* n3, int option)
{
	FileBuilder* builder = (FileBuilder*)handle;
	builder->AddCab(n1, n2, n3, option);
}

