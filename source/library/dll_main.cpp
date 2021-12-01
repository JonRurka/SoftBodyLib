#include "dll_main.h"

using namespace SoftBodyLib;

#define PHYSICS_DT 0.0005f // fixed dt of 0.5 ms

int Exe_Test() {
	return Init();
}

int Init() {
	printf("Init");

	UpdateActors(0, 0.03);

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