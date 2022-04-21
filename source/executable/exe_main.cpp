#include "dll_main.h"
#include "SimContext.h"
#include "terrain/terrain.h"
#include "physics/Physics.h"

using namespace SoftBodyLib;

int main()
{
	SimContext context;


	glm::vec3 val(1, 2, 3);
	float* t = glm::value_ptr(val);
	printf("%f, %f, %f\n", t[0], val[1], val[2]);

	//return 0;

	Collisions_Base* col = new Collisions(glm::vec3(100, 100, 100));
	TerrainManager_Base* terr = new SimpleTerrainManager(col);

	printf("Pointer size: %i\n", (int)sizeof(col));

	TerrainManager_Base::TerrainSettings settings;
	settings.terrain_mgr = terr;
	settings.collisions = col;
	settings.gravity = 0;

	context.LoadTerrain(settings);

	FileBuilder fileBuilder;

	fileBuilder.SetGlobals(100.0f, 0.0f, "");
	fileBuilder.SetNodeDefaults(0, 1, 1, 1, 0);
	fileBuilder.SetBeamDefaults(-1, -1, -1, -1, 4, "mat", 0.2f, false, false, false);
	fileBuilder.SetBeamDefaultsScale(1, 1, 1, 1);


	fileBuilder.AddNode("0", 0, 0, 0, false, 0);
	fileBuilder.AddNode("1", 0, 1, 0, false, 0);
	fileBuilder.AddNode("2", 0, 2, 0, false, 0);

	fileBuilder.AddBeam("0", "1", true, 10000);
	fileBuilder.AddBeam("1", "2", true, 10000);

	ActorSpawnRequest res;

	res.asr_free_position = true;

	res.asr_origin = 2;

	res.asr_rotation = glm::quat(0, 0, 0, 0);

	res.asr_position = glm::vec3(0);

	res.asr_terrn_machine = false;

	Actor* ac = context.SpawnActor(res, &fileBuilder);

	PointColDetector* pointcol = ac->m_intra_point_col_detector;


	/*for (int i = 0; i < 2; i++)
	{
		float* tmp = glm::value_ptr(ac->ar_nodes[i].AbsPosition);
		printf("%f, %f, %f || %f, %f, %f\n", 
			ac->ar_nodes[i].AbsPosition[0], ac->ar_nodes[i].AbsPosition[1], ac->ar_nodes[i].AbsPosition[2],
			tmp[0], tmp[1], tmp[2]);
	}*/

	pointcol->UpdateIntraPoint(true);

	pointcol->query(glm::vec3(-0.2f, 10, 0), glm::vec3(0, 10.2f, 0), glm::vec3(0.2f, 0, 0), 0.1f);



	//Actor_GetNodes(ac)

	//node_t_getAbsPosition(nullptr);


	//return Exe_Test();
}