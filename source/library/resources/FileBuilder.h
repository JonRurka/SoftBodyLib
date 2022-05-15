#pragma once

#include "resources.h"
#include "C_Variables.h"

class FileBuilder
{
public:
	File file;
	NodeDefaults cur_nodeDefaults;
	BeamDefaultsScale cur_beamDefaultsScale;
	BeamDefaults cur_beamDefaults;
	MinimassPreset cur_MinimassPreset;
	Globals globals;
	std::shared_ptr<Submesh> cur_Submesh;

	void SetGlobals(
		float dry_mass,
		float cargo_mass,
		std::string const& material_name);

	void SetNodeDefaults(
		float load_weight,
		float friction,
		float volume,
		float surface,
		unsigned int options);

	void SetBeamDefaultsScale(
		float springiness,
		float damping_constan,
		float deformation_threshold_constant,
		float breaking_threshold_constant);

	void SetBeamDefaults(
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
	);

	void SetMinimassPreset(float min_mass);

	// TODO: Module support
	void AddNode(std::string const& id, float x, float y, float z, bool loadWeight, float weight);

	void AddBeam(std::string const& node_1, std::string const& node_2, bool canBreak, float breakLimit);

	void NewSubmesh();

	void FlushSubmesh();

	void AddCab(std::string const& n1, std::string const& n2, std::string const& n3, int option);

	void AddContacter(std::string const& n1);

	void AddContacters(std::vector<std::string> const& nodes);

private:
	int lineCounter{ 1 };

	Node::Ref getNodeRef(std::string const& node_id)
	{
		return Node::Ref(node_id, 0, Node::Ref::REGULAR_STATE_IS_VALID | Node::Ref::REGULAR_STATE_IS_NAMED, lineCounter);
	}
};



