#include "SimData.h"

using namespace SoftBodyLib;



// #### node_t

C_Vec3   node_t_getRelPosition(void* handle)
{
    node_t* node = (node_t*)handle;
    return C_Vec3::To(node->RelPosition);
}

C_Vec3   node_t_getAbsPosition(void* handle)
{
    node_t* node = (node_t*)handle;
    return C_Vec3::To(node->AbsPosition);
}

C_Vec3   node_t_getVelocity(void* handle)
{
    node_t* node = (node_t*)handle;
    return C_Vec3::To(node->Velocity);
}

C_Vec3   node_t_getForces(void* handle)
{
    node_t* node = (node_t*)handle;
    return C_Vec3::To(node->Forces);
}

// #### collision_box_t

// #### ground_model_t

