#include "ActorSpawner.h"

using namespace SoftBodyLib;

#define PROCESS_SECTION_IN_ANY_MODULE(_FIELD_, _FUNCTION_)			    \
{                                                                       \
    for (auto& m: m_selected_modules)                                   \
    {                                                                   \
        if (m->_FIELD_ != nullptr)                                      \
        {                                                               \
            try {                                                       \
                _FUNCTION_(*(m->_FIELD_));                              \
            }                                                           \
            catch (...)                                                 \
            {                                                           \
                /*this->HandleException();*/                            \
            }                                                           \
            break;                                                      \
        }                                                               \
    }                                                                   \
}																		


#define PROCESS_SECTION_IN_ALL_MODULES(_FIELD_, _FUNCTION_)				\
{                                                                       \
    for (auto& m: m_selected_modules)                                   \
    {                                                                   \
        for (auto& entry: m->_FIELD_)                                   \
        {                                                               \
            try {                                                       \
                _FUNCTION_(entry);                                      \
            }                                                           \
            catch (...)                                                 \
            {                                                           \
                /*this->HandleException();*/                            \
            }                                                           \
        }                                                               \
    }                                                                   \
}

void SoftBodyLib::ActorSpawner::Setup(Actor* actor, std::shared_ptr<File> file, glm::vec3 const& spawn_position)
{
	m_actor = actor;
	m_file = file;
	m_spawn_position = spawn_position;


}

void SoftBodyLib::ActorSpawner::CalcMemoryRequirements(ActorMemoryRequirements& req, File::Module* module_def)
{
	req.num_nodes += module_def->nodes.size();

	req.num_beams += module_def->beams.size();
}

Actor* SoftBodyLib::ActorSpawner::SpawnActor()
{
	InitializeRig();

	m_actor->ar_collision_range = 0.05f;//m_file->collision_range;

	// Section 'gobals' in any module
	PROCESS_SECTION_IN_ANY_MODULE(globals, ProcessGlobals);

	// Sections 'nodes' & 'nodes2'
	PROCESS_SECTION_IN_ALL_MODULES(nodes, ProcessNode);

	// Section 'beams'
	PROCESS_SECTION_IN_ALL_MODULES(beams, ProcessBeam);

	// Section 'submeshes'
	PROCESS_SECTION_IN_ALL_MODULES(submeshes, ProcessSubmesh);

	// Section 'contacters'
	PROCESS_SECTION_IN_ALL_MODULES(contacters, ProcessContacter);

	// Section 'collisionboxes'
	PROCESS_SECTION_IN_ALL_MODULES(collision_boxes, ProcessCollisionBox);

	// Section 'flexbodies' (Uses generated nodes; needs GfxActor to exist)
	PROCESS_SECTION_IN_ALL_MODULES(flexbodies, ProcessFlexbody);


	this->FinalizeRig();

	// Pass ownership
	Actor* rig = m_actor;
	m_actor = nullptr;
	return rig;
}

void SoftBodyLib::ActorSpawner::InitializeRig()
{
	ActorMemoryRequirements& req = m_memory_requirements;
	for (auto module : m_selected_modules) // _Root_ module is included
	{
		this->CalcMemoryRequirements(req, module.get());
	}

	m_actor->ar_beams = new beam_t[req.num_beams];
	m_actor->ar_nodes = new node_t[req.num_nodes];

	m_actor->ar_minimass.resize(req.num_nodes);

	memset(m_actor->ar_collcabs, 0, sizeof(int) * MAX_CABS);
	memset(m_actor->ar_inter_collcabrate, 0, sizeof(collcab_rate_t) * MAX_CABS);

	m_actor->ar_num_collcabs = 0;
	memset(m_actor->ar_intra_collcabrate, 0, sizeof(collcab_rate_t) * MAX_CABS);


	m_actor->m_masscount = 0;

	m_actor->ar_origin = glm::vec3(0, 0, 0);

	m_actor->ar_state = ActorState::LOCAL_SLEEPING;


	/* Collisions */

	if (true) // TODO: sim_no_collisions->getBool()
		m_actor->m_inter_point_col_detector = new PointColDetector(m_actor);

	if (true) // TODO: sim_no_self_collisions->getBool()
		m_actor->m_intra_point_col_detector = new PointColDetector(m_actor);

	m_actor->ar_submesh_ground_model = NULL; // TODO: App::GetSimTerrain()->GetCollisions()->defaultgm;

	m_actor->m_definition = m_file;

	// TODO:
	//m_flex_factory = RoR::FlexFactory(this);
	//m_flex_factory.CheckAndLoadFlexbodyCache();


}

void ActorSpawner::ProcessGlobals(Globals& def)
{
	m_actor->m_dry_mass = def.dry_mass;
	m_actor->m_load_mass = def.cargo_mass;

	// NOTE: Don't do any material pre-processing here; it'll be done on actual entities (via `SetupNewEntity()`).
	if (!def.material_name.empty())
	{
		// TODO: m_cab_material_name = def.material_name;
	}
}

void ActorSpawner::ProcessNode(Node& def)
{
	std::pair<unsigned int, bool> inserted_node = AddNode(def.id);
	if (!inserted_node.second)
	{
		return;
	}

	node_t& node = m_actor->ar_nodes[inserted_node.first];
	node.pos = inserted_node.first;

	/* Positioning */

	glm::vec3 node_position = m_spawn_position + def.position;
	node.AbsPosition = node_position;
	node.RelPosition = node_position - m_actor->ar_origin;

	node.friction_coef = def.node_defaults->friction;
	node.volume_coef = def.node_defaults->volume;
	node.surface_coef = def.node_defaults->surface;

	m_actor->ar_minimass[inserted_node.first] = def.node_minimass->min_mass;

	if (def.node_defaults->load_weight >= 0.0f)
	{
		// orig = further override of hardcoded default.
		node.mass = def.node_defaults->load_weight;
		node.nd_override_mass = true;
		node.nd_loaded_mass = true;
	}
	else
	{
		node.mass = 10; // Hardcoded in original (bts_nodes, call to init_node())
		node.nd_loaded_mass = false;
	}

	/* Lockgroup */
	// TODO: node.nd_lockgroup = (m_file->lockgroup_default_nolock) ? RigDef::Lockgroup::LOCKGROUP_NOLOCK : RigDef::Lockgroup::LOCKGROUP_DEFAULT;

	unsigned int options = def.options | def.node_defaults->options;
	if (false) { // TODO: BITMASK_IS_1(options, RigDef::Node::OPTION_l_LOAD_WEIGHT)
		node.nd_loaded_mass = true;
		if (def.has_load_weight_override)
		{
			node.nd_override_mass = true;
			node.mass = def.load_weight_override;
		}
		else
		{
			m_actor->m_masscount++;
		}
	}

	// hooks and other extentions

	// AdjustNodeBuoyancy(node, def, def.node_defaults);

	node.nd_no_ground_contact = false; // BITMASK_IS_1(options, RigDef::Node::OPTION_c_NO_GROUND_CONTACT);
	node.nd_no_mouse_grab = false; // BITMASK_IS_1(options, RigDef::Node::OPTION_m_NO_MOUSE_GRAB);

}

void ActorSpawner::ProcessBeam(Beam& def)
{
	// Nodes
	node_t* ar_nodes[] = { nullptr, nullptr };
	ar_nodes[0] = GetBeamNodePointer(def.nodes[0]);
	if (ar_nodes[0] == nullptr)
	{
		// Ignoring beam, could not find node
		return;
	}
	ar_nodes[1] = GetBeamNodePointer(def.nodes[1]);
	if (ar_nodes[0] == nullptr)
	{
		// Ignoring beam, could not find node
		return;
	}

	// Beam
	int beam_index = m_actor->ar_num_beams;
	beam_t& beam = AddBeam(*ar_nodes[0], *ar_nodes[1], def.defaults, def.detacher_group);
	beam.bm_type = BeamType::BEAM_NORMAL;
	beam.k = def.defaults->GetScaledSpringiness();
	beam.d = def.defaults->GetScaledDamping();
	beam.bounded = SpecialBeam::NOSHOCK;

	CalculateBeamLength(beam);

	float beam_strength = def.defaults->GetScaledBreakingThreshold();
	beam.strength = beam_strength;

	/* Options */

	if (false) // BITMASK_IS_1(def.options, RigDef::Beam::OPTION_s_SUPPORT)
	{
		beam.bounded = SUPPORTBEAM;
		beam.longbound = def.extension_break_limit;
	}
}

void ActorSpawner::ProcessSubmesh(Submesh& def)
{
	if (!CheckSubmeshLimit(1))
	{
		return;
	}

	/* TEXCOORDS */

	std::vector<Texcoord>::iterator texcoord_itor = def.texcoords.begin();
	for (; texcoord_itor != def.texcoords.end(); texcoord_itor++)
	{
		if (!CheckTexcoordLimit(1))
		{
			break;
		}

		CabTexcoord texcoord;
		texcoord.node_id = GetNodeIndexOrThrow(texcoord_itor->node);
		texcoord.texcoord_u = texcoord_itor->u;
		texcoord.texcoord_v = texcoord_itor->v;
		m_oldstyle_cab_texcoords.push_back(texcoord);
	}

	/* CAB */

	auto cab_itor = def.cab_triangles.begin();
	auto cab_itor_end = def.cab_triangles.end();

	for (; cab_itor != cab_itor_end; ++cab_itor)
	{
		if (!CheckCabLimit(1))
		{
			return;
		}

		if (m_actor->ar_num_collcabs >= MAX_CABS)
		{
			// "Collcab limit (" << MAX_CABS << ") exceeded"
			Logger::LogWarning("ActorSpawner::ProcessSubmesh", "Collcab limit (" + std::to_string(MAX_CABS) + ") exceeded!");
			return;
		}

		bool mk_buoyance = false;

		m_actor->ar_cabs[m_actor->ar_num_cabs * 3] = GetNodeIndexOrThrow(cab_itor->nodes[0]);
		m_actor->ar_cabs[m_actor->ar_num_cabs * 3 + 1] = GetNodeIndexOrThrow(cab_itor->nodes[1]);
		m_actor->ar_cabs[m_actor->ar_num_cabs * 3 + 2] = GetNodeIndexOrThrow(cab_itor->nodes[2]);

		// TODO:
		bool isContact = true;
		if (isContact)
		{
			m_actor->ar_collcabs[m_actor->ar_num_collcabs] = m_actor->ar_num_cabs;
			m_actor->ar_num_collcabs++;
		}

		m_actor->ar_num_cabs++;
		
	}

	// close the current mesh
	CabSubmesh submesh;
	//submesh.texcoords_pos
	submesh.cabs_pos = static_cast<unsigned int>(m_actor->ar_num_cabs);
	submesh.backmesh_type = CabSubmesh::BACKMESH_NONE;
	m_oldstyle_cab_submeshes.push_back(submesh);


	/* BACKMESH */
}

void ActorSpawner::ProcessContacter(Node::Ref& node_ref)
{
	unsigned int node_index = GetNodeIndexOrThrow(node_ref);
	m_actor->ar_nodes[node_index].nd_contacter = true;
}

void ActorSpawner::ProcessCollisionBox(CollisionBox& def)
{
	int8_t bbox_id = static_cast<int8_t>(m_actor->ar_collision_bounding_boxes.size());
	for (Node::Ref& node_ref : def.nodes)
	{
		std::pair<unsigned int, bool> node_result = this->GetNodeIndex(node_ref);
		if (!node_result.second)
		{
			// [RoR|Spawner] Collision box: skipping invalid node 
			continue;
		}
		if (m_actor->ar_nodes[node_result.first].nd_coll_bbox_id != node_t::INVALID_BBOX)
		{
			// [RoR|Spawner] Collision box: re-assigning node '%s' from box ID '%d' to '%d'
		}
		m_actor->ar_nodes[node_result.first].nd_coll_bbox_id = bbox_id;
	}

	m_actor->ar_collision_bounding_boxes.push_back(AxisAlignedBox());
	m_actor->ar_predicted_coll_bounding_boxes.push_back(AxisAlignedBox());


}

void ActorSpawner::ProcessFlexbody(std::shared_ptr<Flexbody> def)
{

}

void ActorSpawner::FinalizeRig()
{



	this->UpdateCollcabContacterNodes();
}

std::pair<unsigned int, bool> ActorSpawner::AddNode(Node::Id& id)
{
	// TODO: logging

	if (!id.IsValid())
	{
		// Attempt to add node with 'INVALID' flag
		printf("Attempt to add node with 'INVALID' flag\n");
		return std::make_pair(0, false);
	}

	// Only supporting named nodes.
	if (id.IsTypeNamed())
	{
		unsigned int new_index = static_cast<unsigned int>(m_actor->ar_num_nodes);
		auto insert_result = m_named_nodes.insert(std::make_pair(id.Str(), new_index));
		if (!insert_result.second)
		{
			// Ignoring named node! Duplicate name
			return std::make_pair(0, false);
		}
		m_actor->ar_num_nodes++;
		printf("Actually added the node!\n");
		return std::make_pair(new_index, true);
	}
	if (id.IsTypeNumbered())
	{
		// Not supported.
		return std::make_pair(0, false);
	}

	// Invalid node ID without type flag!
	//throw Exception("Invalid Node::Id without type flags!");
	return std::make_pair(0, false);
}

std::pair<unsigned int, bool> ActorSpawner::GetNodeIndex(Node::Ref const& node_ref, bool quiet /* Default: false */)
{
	if (!node_ref.IsValidAnyState())
	{
		// Attempt to resolve invalid node reference
		return std::make_pair(0, false);
	}
	bool is_imported = node_ref.GetImportState_IsValid();
	bool is_named = (is_imported ? node_ref.GetImportState_IsResolvedNamed() : node_ref.GetRegularState_IsNamed());

	if (is_named)
	{
		auto result = m_named_nodes.find(node_ref.Str());
		if (result != m_named_nodes.end())
		{
			return std::make_pair(result->second, true);
		}
		else
		{
			// Failed to resolve node-ref (node not found)
		}
		return std::make_pair(0, false);
	}
	else
	{
		// not supported.
		return std::make_pair(0, false);
	}

}

node_t* ActorSpawner::GetNodePointer(Node::Ref const& node_ref)
{
	std::pair<unsigned int, bool> result = GetNodeIndex(node_ref);
	if (result.second)
	{
		return &m_actor->ar_nodes[result.first];
	}
	else
	{
		return nullptr;
	}
}

node_t* ActorSpawner::GetBeamNodePointer(Node::Ref const& node_ref)
{
	node_t* node = GetNodePointer(node_ref);
	if (node != nullptr)
	{
		return node;
	}
	return nullptr;
}

beam_t& ActorSpawner::GetFreeBeam()
{
	beam_t& beam = m_actor->ar_beams[m_actor->ar_num_beams];
	m_actor->ar_num_beams++;
	return beam;
}

beam_t& ActorSpawner::GetAndInitFreeBeam(node_t& node_1, node_t& node_2)
{
	beam_t& beam = GetFreeBeam();
	beam.p1 = &node_1;
	beam.p2 = &node_2;
	return beam;
}

void ActorSpawner::SetBeamDeformationThreshold(beam_t& beam, std::shared_ptr<BeamDefaults> beam_defaults)
{
	// Old init
	float default_deform = BEAM_DEFORM;
	float beam_creak = BEAM_CREAK_DEFAULT;

	// Old 'set_beam_defaults'
	if (beam_defaults->is_user_defined)
	{
		default_deform = beam_defaults->deformation_threshold;
		if (!beam_defaults->enable_advanced_deformation && default_deform < BEAM_DEFORM)
		{
			default_deform = BEAM_DEFORM;
		}

		if (beam_defaults->is_plastic_deform_coef_user_defined && beam_defaults->plastic_deform_coef >= 0.0f)
		{
			beam_creak = 0.0f;
		}
	}

	// Old 'add_beam'
	if (default_deform < beam_creak)
	{
		default_deform = beam_creak;
	}

	float deformation_threshold = default_deform * beam_defaults->scale.deformation_threshold_constant;

	beam.minmaxposnegstress = deformation_threshold;
	beam.maxposstress		= deformation_threshold;
	beam.maxnegstress		= -(deformation_threshold);
}

void ActorSpawner::CalculateBeamLength(beam_t& beam)
{
	float beam_length = glm::length(beam.p1->RelPosition - beam.p2->RelPosition);
	beam.L = beam_length;
	beam.refL = beam_length;
}

beam_t& ActorSpawner::AddBeam(
	node_t& node_1,
	node_t& node_2,
	std::shared_ptr<BeamDefaults>& beam_defaults,
	int detacher_group
)
{
	/* Init */
	beam_t& beam = GetAndInitFreeBeam(node_1, node_2);
	beam.detacher_group = detacher_group;
	beam.bm_disabled = false;

	/* Breaking threshold (strength) */
	float strength = beam_defaults->breaking_threshold;
	beam.strength = strength;

	/* Deformation */
	SetBeamDeformationThreshold(beam, beam_defaults);

	float plastic_coef = beam_defaults->plastic_deform_coef;
	beam.plastic_coef = plastic_coef;

	return beam;
}

unsigned int ActorSpawner::GetNodeIndexOrThrow(Node::Ref const& node_ref)
{
	std::pair<unsigned int, bool> result = GetNodeIndex(node_ref);
	if (!result.second)
	{
		// Failed to retrieve required node
		throw "Failed to retrieve required node";
	}
	return result.first;
}

bool ActorSpawner::CheckSubmeshLimit(unsigned int count)
{
	if ((m_oldstyle_cab_submeshes.size() + count) > MAX_SUBMESHES)
	{
		// "Submesh limit (" << MAX_SUBMESHES << ") exceeded"
		return false;
	}
	return true;
}

bool ActorSpawner::CheckTexcoordLimit(unsigned int count)
{
	if ((m_oldstyle_cab_texcoords.size() + count) > MAX_TEXCOORDS)
	{
		// "Texcoord limit (" << MAX_TEXCOORDS << ") exceeded"
		return false;
	}
	return true;
}

bool ActorSpawner::CheckCabLimit(unsigned int count)
{
	if ((m_actor->ar_num_cabs + count) > MAX_CABS)
	{
		// "Cab limit (" << MAX_CABS << ") exceeded"
		return false;
	}
	return true;
}

void ActorSpawner::UpdateCollcabContacterNodes()
{
	for (int i = 0; i < m_actor->ar_num_collcabs; i++)
	{
		int tmpv = m_actor->ar_collcabs[i] * 3;
		m_actor->ar_nodes[m_actor->ar_cabs[tmpv]].nd_cab_node = true;
		m_actor->ar_nodes[m_actor->ar_cabs[tmpv+1]].nd_cab_node = true;
		m_actor->ar_nodes[m_actor->ar_cabs[tmpv+2]].nd_cab_node = true;
	}

	for (int i = 0; i < m_actor->ar_num_nodes; i++)
	{
		if (m_actor->ar_nodes[i].nd_contacter)
		{
			m_actor->ar_num_contactable_nodes++;
			m_actor->ar_num_contacters++;
		}
		else if (!m_actor->ar_nodes[i].nd_no_ground_contact &&
			(m_actor->ar_nodes[i].nd_cab_node || m_actor->ar_nodes[i].nd_rim_node || m_actor->ar_num_collcabs == 0))
		{
			m_actor->ar_nodes[i].nd_contactable = true;
			m_actor->ar_num_contactable_nodes++;
		}
	}
}



