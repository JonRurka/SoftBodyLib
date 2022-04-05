#pragma once

#include "physics/Physics.h"
#include "resources/resources.h"


extern "C"
{
	int Exe_Test();

	int Init(int);


	// TESTS

	void UpdateActors(float m_dt_remainder, float m_simulation_time);

	bool TestTrisCollision(glm::vec3 t_Pos1, glm::vec3 t_Pos2, glm::vec3 t_Pos3, glm::vec3 testP);



	/* -------------------------------------------------------------------------- */
	/* FileBuilder Interface                                                      */
	/* -------------------------------------------------------------------------- */

	FileBuilder* CreateFileBuilder();

	void FileBuilder_SetGlobals(FileBuilder* file,
		float dry_mass,
		float cargo_mass,
		std::string material_name);

	void FileBuilder_SetNodeDefaults(FileBuilder* builder,
		float load_weight,
		float friction,
		float volume,
		float surface,
		unsigned int options);

	void FileBuilder_SetBeamDefaultsScale(FileBuilder* builder,
		float springiness,
		float damping_constan,
		float deformation_threshold_constant,
		float breaking_threshold_constant);

	void FileBuilder_SetBeamDefaults(FileBuilder* builder,
		float springiness,
		float damping_constant,
		float deformation_threshold,
		float breaking_threshold,
		float visual_beam_diameter,
		std::string beam_material_name,
		float plastic_deform_coef,
		bool _enable_advanced_deformation, //!< Informs whether "enable_advanced_deformation" directive preceded these defaults.
		bool _is_plastic_deform_coef_user_defined,
		bool _is_user_defined //!< Informs whether these data were read from "set_beam_defaults" directive or filled in by the parser on startup.
	);

	void FileBuilder_SetMinimassPreset(FileBuilder* builder, float min_mass);

	void FileBuilder_AddNode(FileBuilder* builder, std::string id, float x, float y, float z, bool loadWeight, float weight);

	void FileBuilder_AddBeam(FileBuilder* builder, std::string const& node_1, std::string const& node_2, bool canBreak, float breakLimit);

	void FileBuilder_NewSubmesh(FileBuilder* builder);

	void FileBuilder_FlushSubmesh(FileBuilder* builder);

	void FileBuilder_AddCab(FileBuilder* builder, std::string const& n1, std::string const& n2, std::string const& n3, int option);
}