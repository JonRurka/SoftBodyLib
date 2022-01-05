#pragma once

#include "stdafx.h"

namespace SoftBodyLib {


	class Actor;
	class ActorManager;
	class ActorSpawner;

	struct CabTexcoord;
	struct CabSubmesh;

	class  FlexBody;
	class  FlexBodyFileIO;
	struct FlexBodyCacheData;
	class  FlexFactory;
	class  FlexMeshWheel;
	class  FlexObj;

	class PointColDetector;

	class ZeroedMemoryAllocator;

	
	
	// SimData.h
	struct node_t;
	struct beam_t;
	struct shock_t;
	struct eventsource_t;

	struct collision_box_t;

	struct ground_model_t;

	namespace Util {
		typedef float Real;

		class Radian;
		class Degree;
		class Angle;
		class Math;

		class Ray;
		class Plane;
		class Sphere;
		class AxisAlignedBox;
	}
}
