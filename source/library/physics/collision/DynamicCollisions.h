#pragma once

#include "../../ForwardDeclarations.h"
#include "../SimData.h"

namespace SoftBodyLib {
    void ResolveInterActorCollisions(const float dt, PointColDetector& interPointCD,
        const int free_collcab, int collcabs[], int cabs[],
        collcab_rate_t inter_collcabrate[], node_t nodes[],
        const float collrange,
        ground_model_t& submesh_ground_model);

    void ResolveIntraActorCollisions(const float dt, PointColDetector& intraPointCD,
        const int free_collcab, int collcabs[], int cabs[],
        collcab_rate_t intra_collcabrate[], node_t nodes[],
        const float collrange,
        ground_model_t& submesh_ground_model);
}