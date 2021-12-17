#include "dll_main.h"

using namespace SoftBodyLib;

#define PHYSICS_DT 0.0005f // fixed dt of 0.5 ms

int lineCounter = 0;

int Exe_Test() {
	return Init();
}

int Init() {
	printf("Init\n");

	//UpdateActors(0, 0.03);

	auto a = glm::vec3(0, 0, 0);
	auto b = glm::vec3(0.5, 1, 0);
	auto c = glm::vec3(1, 0, 0);

	auto t = glm::vec3(0.5, 0.5, 0);

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

FileBuilder* CreateFileBuilder()
{
	FileBuilder* builder = new FileBuilder();
	lineCounter++;
	return builder;
}

void FileBuilder_SetGlobals(FileBuilder* builder,
	float dry_mass,
	float cargo_mass,
	std::string material_name)
{
	builder->SetGlobals(dry_mass, cargo_mass, material_name);
}

void FileBuilder_SetNodeDefaults(FileBuilder* builder,
	float load_weight,
	float friction,
	float volume,
	float surface,
	unsigned int options)
{
	builder->SetNodeDefaults(load_weight, friction, volume, surface, options);
}

void FileBuilder_SetBeamDefaultsScale(FileBuilder* builder,
	float springiness,
	float damping_constan,
	float deformation_threshold_constant,
	float breaking_threshold_constant)
{
	builder->SetBeamDefaultsScale(springiness, damping_constan, deformation_threshold_constant, breaking_threshold_constant);
}

void FileBuilder_SetBeamDefaults(FileBuilder* builder,
	float springiness,
	float damping_constant,
	float deformation_threshold,
	float breaking_threshold,
	float visual_beam_diameter,
	std::string beam_material_name,
	float plastic_deform_coef,
	bool enable_advanced_deformation, //!< Informs whether "enable_advanced_deformation" directive preceded these defaults.
	bool is_plastic_deform_coef_user_defined,
	bool is_user_defined //!< Informs whether these data were read from "set_beam_defaults" directive or filled in by the parser on startup.
)
{
	builder->SetBeamDefaults(
		springiness,
		damping_constant,
		deformation_threshold,
		breaking_threshold,
		visual_beam_diameter,
		beam_material_name,
		plastic_deform_coef,
		enable_advanced_deformation,
		is_plastic_deform_coef_user_defined,
		is_user_defined
	);
}

void FileBuilder_SetMinimassPreset(FileBuilder* builder, float min_mass)
{
	builder->SetMinimassPreset(min_mass);
}

// TODO: Module support
void FileBuilder_AddNode(FileBuilder* builder, std::string id, float x, float y, float z, bool loadWeight, float weight)
{
	builder->AddNode(id, x, y, z, loadWeight, weight);
}

void FileBuilder_AddBeam(FileBuilder* builder, std::string const& node_1, std::string const& node_2, bool canBreak, float breakLimit)
{
	builder->AddBeam(node_1, node_2, canBreak, breakLimit);
}

void FileBuilder_NewSubmesh(FileBuilder* builder)
{
	builder->NewSubmesh();
}

void FileBuilder_FlushSubmesh(FileBuilder* builder)
{
	builder->FlushSubmesh();
}

void FileBuilder_AddCab(FileBuilder* builder, std::string const& n1, std::string const& n2, std::string const& n3, int option)
{
	builder->AddCab(n1, n2, n3, option);
}
