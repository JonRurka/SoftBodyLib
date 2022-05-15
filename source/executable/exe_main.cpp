#include "dll_main.h"
#include "SimContext.h"
#include "terrain/terrain.h"
#include "physics/Physics.h"

using namespace SoftBodyLib;

struct test_strct {
	int tst_1;
	float tst_2;
	unsigned long tst_3;
	void* ptr;
};

int main()
{







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
	//fileBuilder.AddNode("2", 0, 2, 0, false, 0);

	fileBuilder.AddBeam("0", "1", true, 10000);
	//fileBuilder.AddBeam("1", "2", true, 10000);

	ActorSpawnRequest res;

	res.asr_free_position = true;

	res.asr_origin = 2;

	res.asr_rotation = glm::quat(0, 0, 0, 0);

	res.asr_position = glm::vec3(0);

	res.asr_terrn_machine = false;

	Actor* ac = context.SpawnActor(res, &fileBuilder);

	PointColDetector* pointcol = ac->m_intra_point_col_detector;

	//node_t n = ac->ar_nodes[1];
	//printf("node 1: %f, %f, %f\n", n.AbsPosition.x, n.AbsPosition.y, n.AbsPosition.z);




	pointcol->UpdateIntraPoint(true);

	pointcol->query(glm::vec3(-0.2f, 0, 0), glm::vec3(0, 1.2f, 0), glm::vec3(0.2f, 0, 0), 0.1f);

	for (auto hit : pointcol->hit_list)
	{
		node_t* n = hit->node;
		printf("hit node: %f, %f, %f\n", n->AbsPosition.x, n->AbsPosition.y, n->AbsPosition.z);
	}

	//Actor_GetNodes(ac)

	//node_t_getAbsPosition(nullptr);


	//return Exe_Test();
}