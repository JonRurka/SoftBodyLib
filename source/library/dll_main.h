#pragma once

#include "physics/Physics.h"

extern "C"
{
	int Exe_Test();

	int Init();

	void UpdateActors(float m_dt_remainder, float m_simulation_time);
}