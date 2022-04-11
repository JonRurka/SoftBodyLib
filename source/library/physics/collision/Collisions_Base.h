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

}