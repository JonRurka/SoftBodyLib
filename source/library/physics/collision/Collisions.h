#pragma once

#include "../Physics.h"


#ifdef ZERO_MEM_ALLOC
#include "../../utils/ZeroedMemoryAllocator.h"
#endif


namespace SoftBodyLib {

    struct eventsource_t
    {
        char instancename[256];
        char boxname[256];
        //Ogre::SceneNode* snode;
        glm::quat direction;
        int scripthandler;
        int cbox;
        bool enabled;
    };

    class Collisions 
#ifdef ZERO_MEM_ALLOC
        : public ZeroedMemoryAllocator 
#endif
    {
        enum SurfaceType
        {
            FX_NONE,
            FX_HARD, // hard surface: rubber burning and sparks
            FX_DUSTY, // dusty surface (with dust colour)
            FX_CLUMPY, // throws clumps (e.g. snow, grass) with colour
            FX_PARTICLE
        };

    public:
        Collisions(glm::vec3 terrn_size);
        ~Collisions();

    private:

        /// Static collision object lookup system
        /// -------------------------------------
        /// Terrain is split into equal-size 'cells' of dimension CELL_SIZE, identified by CellID
        /// A hash table aggregates elements from multiple cells in one entry
        struct hash_coll_element_t
        {
            static const int ELEMENT_TRI_BASE_INDEX = 1000000; // Effectively a maximum number of collision boxes

            inline hash_coll_element_t(unsigned int cell_id_, int value) : cell_id(cell_id_), element_index(value) {}

            inline bool IsCollisionBox() const { return element_index < ELEMENT_TRI_BASE_INDEX; }
            inline bool IsCollisionTri() const { return element_index >= ELEMENT_TRI_BASE_INDEX; }

            unsigned int cell_id;

            /// Values below ELEMENT_TRI_BASE_INDEX are collision box indices (Collisions::m_collision_boxes),
            ///    values above are collision tri indices (Collisions::m_collision_tris).
            int element_index;
        };

        struct collision_tri_t
        {
            glm::vec3 a;
            glm::vec3 b;
            glm::vec3 c;
            //Ogre::AxisAlignedBox aab; // TODO
            glm::mat3x3 forward;
            glm::mat3x3 reverse;
            ground_model_t* gm;
            bool enabled;
        };

        static const int LATEST_GROUND_MODEL_VERSION = 3;
        static const int MAX_EVENT_SOURCE = 500;

        // this is a power of two, change with caution
        static const int HASH_POWER = 20;
        static const int HASH_SIZE = 1 << HASH_POWER;

        // how many elements per cell? power of 2 minus 2 is better
        static const int CELL_BLOCKSIZE = 126;

        // terrain size is limited to 327km x 327km:
        static const int CELL_SIZE = 2.0; // we divide through this
        static const int MAXIMUM_CELL = 0x7FFF;

        // collision boxes pool
        std::vector<collision_box_t> m_collision_boxes; // Formerly MAX_COLLISION_BOXES = 5000
        std::vector<collision_box_t*> m_last_called_cboxes;

        // collision tris pool;
        std::vector<collision_tri_t> m_collision_tris; // Formerly MAX_COLLISION_TRIS = 100000

        // TODO:
        //Ogre::AxisAlignedBox m_collision_aab; // Tight bounding box around all collision meshes 

        // collision hashtable
        float hashtable_height[HASH_SIZE];
        std::vector<hash_coll_element_t> hashtable[HASH_SIZE];

        // ground models
        std::map<std::string, ground_model_t> ground_models;

        // event sources
        eventsource_t eventsources[MAX_EVENT_SOURCE];
        int free_eventsource;

        bool permitEvent(CollisionEventFilter filter);
        bool envokeScriptCallback(collision_box_t* cbox, node_t* node = 0);

        // TODO:
        //Landusemap* landuse;
        //Ogre::ManualObject* debugmo;
        bool debugMode;
        int collision_version;
        inline int GetNumCollisionTris() const { return static_cast<int>(m_collision_tris.size()); }
        inline int GetNumCollisionBoxes() const { return static_cast<int>(m_collision_boxes.size()); }
        unsigned int hashmask;

        const glm::vec3 m_terrain_size;

        void hash_add(int cell_x, int cell_z, int value, float h);
        int hash_find(int cell_x, int cell_z); /// Returns index to 'hashtable'
        unsigned int hashfunc(unsigned int cellid);

        // TODO:
        void parseGroundConfig(void* cfg, std::string groundModel = "");

        glm::vec3 calcCollidedSide(const glm::vec3& pos, const glm::vec3& lo, const glm::vec3& hi);

     public:
         std::mutex m_scriptcallback_mutex;

         bool forcecam;
         glm::vec3 forcecampos;
         ground_model_t* defaultgm, * defaultgroundgm;

         glm::vec3 getPosition(const std::string& inst, const std::string& box);
         glm::quat getDirection(const std::string& inst, const std::string& box);
         collision_box_t* getBox(const std::string& inst, const std::string& box);

         // TODO:
         std::pair<bool, SoftBodyLib::Util::Real> intersectsTris(SoftBodyLib::Util::Ray ray);

         float getSurfaceHeight(float x, float z);
         float getSurfaceHeightBelow(float x, float z, float height);
         bool collisionCorrect(glm::vec3* refpos, bool envokeScriptCallbacks = true);
         bool groundCollision(node_t* node, float dt); // 1st in Actor::CalcNodes()
         bool isInside(glm::vec3 pos, const std::string& inst, const std::string& box, float border = 0);
         bool isInside(glm::vec3 pos, collision_box_t* cbox, float border = 0);
         bool nodeCollision(node_t* node, float dt, bool envokeScriptCallbacks = true); // 2nd/3rd in Actor::CalcNodes()

         void finishLoadingTerrain();

         int addCollisionBox(/*Ogre::SceneNode* tenode, */ 
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
                             int scripthandler = -1);

         int addCollisionMesh(std::string meshname, 
                              glm::vec3 pos, 
                              glm::quat q, 
                              glm::vec3 scale, 
                              ground_model_t* gm = 0, 
                              std::vector<int>* collTris = 0);

         int addCollisionTri(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, ground_model_t* gm);
         int createCollisionDebugVisualization();
         void removeCollisionBox(int number);
         void removeCollisionTri(int number);
         void clearEventCache() { m_last_called_cboxes.clear(); }

         // TODO:
         //Ogre::AxisAlignedBox getCollisionAAB() { return m_collision_aab; };

         // ground models things
         int loadDefaultModels();
         int loadGroundModelsConfigFile(std::string filename);
         std::map<std::string, ground_model_t>* getGroundModels() { return &ground_models; };
         void setupLandUse(const char* configfile);
         ground_model_t* getGroundModelByString(const std::string name);

         // TODO: Mesh
         void getMeshInformation(/*Ogre::Mesh*/ void* mesh, size_t& vertex_count, glm::vec3*& vertices,
             size_t& index_count, unsigned*& indices,
             const glm::vec3& position = glm::vec3(0, 0, 0),
             const glm::quat& orient = glm::quat(1, 0, 0, 0), const glm::vec3& scale = glm::vec3(1, 1, 1));
    };

    glm::vec3 primitiveCollision(node_t* node, glm::vec3 velocity, float mass, glm::vec3 normal, float dt, ground_model_t* gm, float penetration = 0);
}