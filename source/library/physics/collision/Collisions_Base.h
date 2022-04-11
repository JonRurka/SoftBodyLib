#pragma once

#include "physics/Physics.h"

namespace SoftBodyLib {

    class Collisions_Base
    {
    public:
        virtual std::pair<bool, SoftBodyLib::Util::Real> intersectsTris(SoftBodyLib::Util::Ray ray) = 0;

        virtual float getSurfaceHeight(float x, float z) = 0;
        virtual float getSurfaceHeightBelow(float x, float z, float height) = 0;
        virtual bool collisionCorrect(glm::vec3* refpos, bool envokeScriptCallbacks = true) = 0;
        virtual bool groundCollision(SoftBodyLib::node_t* node, float dt) = 0; // 1st in Actor::CalcNodes()
        virtual bool isInside(glm::vec3 pos, const std::string& inst, const std::string& box, float border = 0) = 0;
        virtual bool isInside(glm::vec3 pos, SoftBodyLib::collision_box_t* cbox, float border = 0) = 0;
        virtual bool nodeCollision(SoftBodyLib::node_t* node, float dt, bool envokeScriptCallbacks = true) = 0;

        virtual void finishLoadingTerrain() = 0;

        virtual int addCollisionBox(/*Ogre::SceneNode* tenode, */
            bool rotating,
            bool virt,
            glm::vec3 pos,
            glm::vec3 rot,
            glm::vec3 l,
            glm::vec3 h,
            glm::vec3 sr,
            const std::string& eventname,
            const std::string& instancename,
            bool forcecam,
            glm::vec3 campos,
            glm::vec3 sc = glm::vec3(1, 1, 1),
            glm::vec3 dr = glm::vec3(0, 0, 0),
            CollisionEventFilter event_filter = EVENT_ALL,
            int scripthandler = -1) = 0;

        virtual int addCollisionMesh(std::string meshname,
            glm::vec3 pos,
            glm::quat q,
            glm::vec3 scale,
            ground_model_t* gm = 0,
            std::vector<int>* collTris = 0) = 0;

        virtual int addCollisionTri(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, ground_model_t* gm) = 0;

        virtual void removeCollisionBox(int number) = 0;
        virtual void removeCollisionTri(int number) = 0;
        virtual void clearEventCache() = 0;

        virtual SoftBodyLib::Util::AxisAlignedBox getCollisionAAB() = 0;


    };

    glm::vec3 primitiveCollision(node_t* node, glm::vec3 velocity, float mass, glm::vec3 normal, float dt, ground_model_t* gm, float penetration = 0);
};

extern "C"
{
    float Collisions_Base_getSurfaceHeight(void* handle, float x, float z);

    float Collisions_Base_getSurfaceHeightBelow(void* handle, float x, float z, float height);

    //bool Collisions_Base_collisionCorrect(void* handle, glm::vec3* refpos, bool envokeScriptCallbacks = true);

    bool Collisions_Base_groundCollision(void* handle, void* node, float dt);

    bool Collisions_Base_isInside_1(void* handle, C_Vec3 pos, const std::string& inst, const std::string& box, float border = 0);

    bool Collisions_Base_isInside_2(void* handle, C_Vec3 pos, void* cbox, float border = 0);

    bool Collisions_Base_nodeCollision(void* handle, void* node, float dt);

    int Collisions_Base_addCollisionBox(void* handle,
        bool rotating,
        bool virt,
        C_Vec3 pos,
        C_Vec3 rot,
        C_Vec3 l,
        C_Vec3 h,
        C_Vec3 sr,
        const std::string& eventname,
        const std::string& instancename,
        bool forcecam,
        C_Vec3 campos,
        C_Vec3 sc,
        C_Vec3 dr,
        short event_filter,
        int scripthandler);

    int Collisions_Base_addCollisionTri(void* handle, C_Vec3 p1, C_Vec3 p2, C_Vec3 p3, void* gm);

    void Collisions_Base_removeCollisionBox(void* handle, int number);

    void Collisions_Base_removeCollisionTri(void* handle, int number);

    void Collisions_Base_clearEventCache(void* handle);

    void* Collisions_Base_getCollisionAAB(void* handle);

    C_Vec3 C_primitiveCollision(void* node, C_Vec3 velocity, float mass, C_Vec3 normal, float dt, void* gm, float penetration);
}