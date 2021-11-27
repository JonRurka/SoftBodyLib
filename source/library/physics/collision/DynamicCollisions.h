#pragma once

#include "../Physics.h"

namespace SoftBodyLib {
    static bool BackfaceCollisionTest(const float distance,
        const glm::vec3& normal,
        const node_t& surface_point,
        const std::vector<int>& neighbour_node_ids,
        const node_t nodes[]);

    static bool InsideTriangleTest(const CartesianToTriangleTransform::TriangleCoord& local, const float margin);

    void ResolveCollisionForces(const float penetration_depth,
        node_t& hitnode, node_t& na, node_t& nb, node_t& no,
        const float alpha, const float beta, const float gamma,
        const glm::vec3& normal,
        const float dt,
        const bool remote,
        ground_model_t& submesh_ground_model);

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