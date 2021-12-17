#include "FileBuilder.h"


void FileBuilder::SetGlobals(
	float dry_mass,
	float cargo_mass,
	std::string const& material_name)
{
	Globals globs;
    globs.dry_mass = dry_mass;
    globs.cargo_mass = cargo_mass;
    globs.material_name = material_name;


	file.root_module->globals = std::make_shared<Globals>(globs);

	lineCounter++;
}

void FileBuilder::SetNodeDefaults(
	float load_weight,
	float friction,
	float volume,
	float surface,
	unsigned int options)
{
	NodeDefaults defaults;
	defaults.load_weight = load_weight;
	defaults.friction = friction;
	defaults.volume = volume;
	defaults.surface = surface;
	defaults.options = options;

	cur_nodeDefaults = defaults;


	lineCounter++;
}

void FileBuilder::SetBeamDefaultsScale(
	float springiness,
	float damping_constan,
	float deformation_threshold_constant,
	float breaking_threshold_constant)
{
	BeamDefaultsScale defaults;
	defaults.springiness = springiness;
	defaults.damping_constant = damping_constan;
	defaults.deformation_threshold_constant = deformation_threshold_constant;
	defaults.breaking_threshold_constant = breaking_threshold_constant;

	cur_beamDefaultsScale = defaults;


	lineCounter++;
}

void FileBuilder::SetBeamDefaults(
	float springiness,
	float damping_constant,
	float deformation_threshold,
	float breaking_threshold,
	float visual_beam_diameter,
	std::string const& beam_material_name,
	float plastic_deform_coef,
	bool enable_advanced_deformation, //!< Informs whether "enable_advanced_deformation" directive preceded these defaults.
	bool is_plastic_deform_coef_user_defined,
	bool is_user_defined //!< Informs whether these data were read from "set_beam_defaults" directive or filled in by the parser on startup.
)
{
	BeamDefaults defaults;
	defaults.springiness = springiness;
	defaults.damping_constant = damping_constant;
	defaults.deformation_threshold = deformation_threshold;
	defaults.breaking_threshold = breaking_threshold;
	defaults.visual_beam_diameter = visual_beam_diameter;
	defaults.beam_material_name = beam_material_name;
	defaults.plastic_deform_coef = plastic_deform_coef;
	defaults.enable_advanced_deformation = enable_advanced_deformation;
	defaults.is_plastic_deform_coef_user_defined = is_plastic_deform_coef_user_defined;
	defaults.is_user_defined = is_user_defined;
	defaults.scale = cur_beamDefaultsScale;

	cur_beamDefaults = defaults;

	lineCounter++;
}

void FileBuilder::SetMinimassPreset(float min_mass)
{
	cur_MinimassPreset = MinimassPreset{ min_mass };
	lineCounter++;
}

// TODO: Module support
void FileBuilder::AddNode(std::string const& id, float x, float y, float z, bool loadWeight, float weight)
{
	Node node;
	node.node_defaults = std::make_shared<NodeDefaults>(cur_nodeDefaults);
	node.beam_defaults = std::make_shared<BeamDefaults>(cur_beamDefaults);
	node.node_minimass = std::make_shared<MinimassPreset>(cur_MinimassPreset);
	node.detacher_group = 0; // TODO

	node.position.x = x;
	node.position.y = y;
	node.position.z = z;

	if (loadWeight)
	{
		node.load_weight_override = weight;
		node.has_load_weight_override = true;
	}

	file.root_module->nodes.push_back(node);

	lineCounter++;
}

void FileBuilder::AddBeam(std::string const& node_1, std::string const& node_2, bool canBreak, float breakLimit)
{
	Beam beam;
	beam.defaults = std::make_shared<BeamDefaults>(cur_beamDefaults);
	beam.detacher_group = 0; // TODO

	beam.nodes[0] = getNodeRef(node_1);
	beam.nodes[1] = getNodeRef(node_2);

	lineCounter++;

	float support_break_limit = 0.0f;
	float support_break_factor = breakLimit;
	if (support_break_factor > 0.0f)
	{
		support_break_limit = support_break_factor;
	}
	beam.extension_break_limit = support_break_limit;
	beam.has_extension_break_limit = true;

	file.root_module->beams.push_back(beam);
}

void FileBuilder::NewSubmesh() {
	cur_Submesh = std::make_shared<Submesh>();
	lineCounter++;
}

void FileBuilder::FlushSubmesh() {
	file.root_module->submeshes.push_back(*cur_Submesh);
	cur_Submesh = nullptr;
	lineCounter++;
}

void FileBuilder::AddCab(std::string const& n1, std::string const& n2, std::string const& n3, int option)
{
	if (cur_Submesh == nullptr)
		return;

	Cab cab;
	cab.nodes[0] = getNodeRef(n1);
	cab.nodes[1] = getNodeRef(n2);
	cab.nodes[2] = getNodeRef(n3);
	cab.options = option;

	cur_Submesh->cab_triangles.push_back(cab);
	lineCounter++;
}
