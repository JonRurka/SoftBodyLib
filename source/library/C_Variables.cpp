#include "C_Variables.h"

C_Vec2 C_Vec2::To(glm::vec2 o)
{
    C_Vec2 res;

    res.x = o.x;
    res.y = o.y;

    return res;
}

glm::vec2 C_Vec2::From(C_Vec2 o)
{
    glm::vec2 res;

    res.x = o.x;
    res.y = o.y;

    return res;
}


C_Vec3 C_Vec3::To(glm::vec3 o)
{
    C_Vec3 res;

    res.x = o.x;
    res.y = o.y;
    res.z = o.z;

    return res;
}

glm::vec3 C_Vec3::From(C_Vec3 o)
{
    glm::vec3 res;

    res.x = o.x;
    res.y = o.y;
    res.z = o.z;

    return res;
}


C_Vec4 C_Vec4::To(glm::vec4 o)
{
    C_Vec4 res;

    res.x = o.x;
    res.y = o.y;
    res.z = o.z;
    res.w = o.w;

    return res;
}

glm::vec4 C_Vec4::From(C_Vec4 o)
{
    glm::vec4 res;

    res.x = o.x;
    res.y = o.y;
    res.z = o.z;
    res.w = o.w;

    return res;
}


C_Quat C_Quat::To(glm::quat o)
{
    C_Quat res;

    res.x = o.x;
    res.y = o.y;
    res.z = o.z;
    res.w = o.w;

    return res;
}

glm::quat C_Quat::From(C_Quat o)
{
    glm::quat res;

    res.x = o.x;
    res.y = o.y;
    res.z = o.z;
    res.w = o.w;

    return res;
}
