#include "../Physics.h"

using namespace SoftBodyLib;
using namespace SoftBodyLib::Util;

// some gcc fixes
#ifdef __linux__
#pragma GCC diagnostic ignored "-Wfloat-equal"
#endif //__linux__

//hash function SBOX
//from http://home.comcast.net/~bretm/hash/10.html
unsigned int sbox[] =
{
    0xF53E1837, 0x5F14C86B, 0x9EE3964C, 0xFA796D53,
    0x32223FC3, 0x4D82BC98, 0xA0C7FA62, 0x63E2C982,
    0x24994A5B, 0x1ECE7BEE, 0x292B38EF, 0xD5CD4E56,
    0x514F4303, 0x7BE12B83, 0x7192F195, 0x82DC7300,
    0x084380B4, 0x480B55D3, 0x5F430471, 0x13F75991,
    0x3F9CF22C, 0x2FE0907A, 0xFD8E1E69, 0x7B1D5DE8,
    0xD575A85C, 0xAD01C50A, 0x7EE00737, 0x3CE981E8,
    0x0E447EFA, 0x23089DD6, 0xB59F149F, 0x13600EC7,
    0xE802C8E6, 0x670921E4, 0x7207EFF0, 0xE74761B0,
    0x69035234, 0xBFA40F19, 0xF63651A0, 0x29E64C26,
    0x1F98CCA7, 0xD957007E, 0xE71DDC75, 0x3E729595,
    0x7580B7CC, 0xD7FAF60B, 0x92484323, 0xA44113EB,
    0xE4CBDE08, 0x346827C9, 0x3CF32AFA, 0x0B29BCF1,
    0x6E29F7DF, 0xB01E71CB, 0x3BFBC0D1, 0x62EDC5B8,
    0xB7DE789A, 0xA4748EC9, 0xE17A4C4F, 0x67E5BD03,
    0xF3B33D1A, 0x97D8D3E9, 0x09121BC0, 0x347B2D2C,
    0x79A1913C, 0x504172DE, 0x7F1F8483, 0x13AC3CF6,
    0x7A2094DB, 0xC778FA12, 0xADF7469F, 0x21786B7B,
    0x71A445D0, 0xA8896C1B, 0x656F62FB, 0x83A059B3,
    0x972DFE6E, 0x4122000C, 0x97D9DA19, 0x17D5947B,
    0xB1AFFD0C, 0x6EF83B97, 0xAF7F780B, 0x4613138A,
    0x7C3E73A6, 0xCF15E03D, 0x41576322, 0x672DF292,
    0xB658588D, 0x33EBEFA9, 0x938CBF06, 0x06B67381,
    0x07F192C6, 0x2BDA5855, 0x348EE0E8, 0x19DBB6E3,
    0x3222184B, 0xB69D5DBA, 0x7E760B88, 0xAF4D8154,
    0x007A51AD, 0x35112500, 0xC9CD2D7D, 0x4F4FB761,
    0x694772E3, 0x694C8351, 0x4A7E3AF5, 0x67D65CE1,
    0x9287DE92, 0x2518DB3C, 0x8CB4EC06, 0xD154D38F,
    0xE19A26BB, 0x295EE439, 0xC50A1104, 0x2153C6A7,
    0x82366656, 0x0713BC2F, 0x6462215A, 0x21D9BFCE,
    0xBA8EACE6, 0xAE2DF4C1, 0x2A8D5E80, 0x3F7E52D1,
    0x29359399, 0xFEA1D19C, 0x18879313, 0x455AFA81,
    0xFADFE838, 0x62609838, 0xD1028839, 0x0736E92F,
    0x3BCA22A3, 0x1485B08A, 0x2DA7900B, 0x852C156D,
    0xE8F24803, 0x00078472, 0x13F0D332, 0x2ACFD0CF,
    0x5F747F5C, 0x87BB1E2F, 0xA7EFCB63, 0x23F432F0,
    0xE6CE7C5C, 0x1F954EF6, 0xB609C91B, 0x3B4571BF,
    0xEED17DC0, 0xE556CDA0, 0xA7846A8D, 0xFF105F94,
    0x52B7CCDE, 0x0E33E801, 0x664455EA, 0xF2C70414,
    0x73E7B486, 0x8F830661, 0x8B59E826, 0xBB8AEDCA,
    0xF3D70AB9, 0xD739F2B9, 0x4A04C34A, 0x88D0F089,
    0xE02191A2, 0xD89D9C78, 0x192C2749, 0xFC43A78F,
    0x0AAC88CB, 0x9438D42D, 0x9E280F7A, 0x36063802,
    0x38E8D018, 0x1C42A9CB, 0x92AAFF6C, 0xA24820C5,
    0x007F077F, 0xCE5BC543, 0x69668D58, 0x10D6FF74,
    0xBE00F621, 0x21300BBE, 0x2E9E8F46, 0x5ACEA629,
    0xFA1F86C7, 0x52F206B8, 0x3EDF1A75, 0x6DA8D843,
    0xCF719928, 0x73E3891F, 0xB4B95DD6, 0xB2A42D27,
    0xEDA20BBF, 0x1A58DBDF, 0xA449AD03, 0x6DDEF22B,
    0x900531E6, 0x3D3BFF35, 0x5B24ABA2, 0x472B3E4C,
    0x387F2D75, 0x4D8DBA36, 0x71CB5641, 0xE3473F3F,
    0xF6CD4B7F, 0xBF7D1428, 0x344B64D0, 0xC5CDFCB6,
    0xFE2E0182, 0x2C37A673, 0xDE4EB7A3, 0x63FDC933,
    0x01DC4063, 0x611F3571, 0xD167BFAF, 0x4496596F,
    0x3DEE0689, 0xD8704910, 0x7052A114, 0x068C9EC5,
    0x75D0E766, 0x4D54CC20, 0xB44ECDE2, 0x4ABC653E,
    0x2C550A21, 0x1A52C0DB, 0xCFED03D0, 0x119BAFE2,
    0x876A6133, 0xBC232088, 0x435BA1B2, 0xAE99BBFA,
    0xBB4F08E4, 0xA62B5F49, 0x1DA4B695, 0x336B84DE,
    0xDC813D31, 0x00C134FB, 0x397A98E6, 0x151F0E64,
    0xD9EB3E69, 0xD3C7DF60, 0xD2F2C336, 0x2DDD067B,
    0xBD122835, 0xB0B3BD3A, 0xB0D54E46, 0x8641F1E4,
    0xA0B38F96, 0x51D39199, 0x37A6AD75, 0xDF84EE41,
    0x3C034CBA, 0xACDA62FC, 0x11923B8B, 0x45EF170A,
};


Collisions::Collisions(glm::vec3 terrn_size) :
    debugMode(false)
    //, debugmo(nullptr)
    , forcecam(false)
    , free_eventsource(0)
    , hashmask(0)
    //, landuse(0)
    , m_terrain_size(terrn_size)
{

}

Collisions::~Collisions()
{

}

int Collisions::loadDefaultModels() 
{
    return 0;
}

int Collisions::loadGroundModelsConfigFile(std::string filename) {
    return 0;
}

void Collisions::parseGroundConfig(void* cfg, std::string groundModel)
{

}

glm::vec3 Collisions::calcCollidedSide(const glm::vec3& pos, const glm::vec3& lo, const glm::vec3& hi)
{
    return glm::vec3(0);
}

void Collisions::setupLandUse(const char* configfile) {

}

void Collisions::removeCollisionBox(int number)
{

}

void Collisions::removeCollisionTri(int number)
{

}

ground_model_t* Collisions::getGroundModelByString(const std::string name)
{
    return nullptr;
}

unsigned int Collisions::hashfunc(unsigned int cellid)
{
    return 0;
}

void Collisions::hash_add(int cell_x, int cell_z, int value, float h)
{

}

int Collisions::hash_find(int cell_x, int cell_z)
{
    return 0;
}

int Collisions::addCollisionBox(
    /*Ogre::SceneNode* tenode, */ 
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
    glm::vec3 sc /* = Vector3::UNIT_SCALE */,
    glm::vec3 dr /* = Vector3::ZERO */,
    CollisionEventFilter event_filter /* = EVENT_ALL */, 
    int scripthandler /* = -1 */)
{
    return 0;
}

int Collisions::addCollisionTri(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, ground_model_t* gm)
{
    return 0;
}

bool Collisions::envokeScriptCallback(collision_box_t* cbox, node_t* node)
{
    return false;
}

std::pair<bool, Real> Collisions::intersectsTris(Ray ray)
{
    return std::pair<bool, Real>();
}

float Collisions::getSurfaceHeight(float x, float z)
{
    return 0;
}

float Collisions::getSurfaceHeightBelow(float x, float z, float height)
{
    return 0;
}

bool Collisions::collisionCorrect(glm::vec3* refpos, bool envokeScriptCallbacks)
{
    return false;
}

bool Collisions::permitEvent(CollisionEventFilter filter)
{
    return false;
}

// 2nd/3rd in Actor::CalcNodes()
bool Collisions::nodeCollision(node_t* node, float dt, bool envokeScriptCallbacks)
{
    return false;
}

glm::vec3 Collisions::getPosition(const std::string& inst, const std::string& box)
{
    return glm::vec3(0);
}

glm::quat Collisions::getDirection(const std::string& inst, const std::string& box)
{
    return glm::quat();
}

collision_box_t* Collisions::getBox(const std::string& inst, const std::string& box)
{
    return nullptr;
}

bool Collisions::isInside(glm::vec3 pos, const std::string& inst, const std::string& box, float border)
{
    return false;
}

bool Collisions::isInside(glm::vec3 pos, collision_box_t* cbox, float border)
{
    return false;
}

// 1st in Actor::CalcNodes()
bool Collisions::groundCollision(node_t* node, float dt)
{
    Real v = 0; // TODO: get height or ISO val: App::GetSimTerrain()->GetHeightAt(node->AbsPosition.x, node->AbsPosition.z);
    if (v > node->AbsPosition.y)
    {
        ground_model_t* ogm = nullptr; // landuse ? landuse->getGroundModelAt(node->AbsPosition.x, node->AbsPosition.z) : nullptr;
        // when landuse fails or we don't have it, use the default value
        if (!ogm)
            ogm = defaultgroundgm;
        glm::vec3 normal(0, 1, 0); //App::GetSimTerrain()->GetNormalAt(node->AbsPosition.x, v, node->AbsPosition.z);
        node->Forces += primitiveCollision(node, node->Velocity, node->mass, normal, dt, ogm, v - node->AbsPosition.y);
        node->nd_last_collision_gm = ogm;
        return true;
    }
    return false;



    return false;
}

glm::vec3 SoftBodyLib::primitiveCollision(node_t* node, glm::vec3 velocity, float mass, glm::vec3 normal, float dt, ground_model_t* gm, float penetration)
{
    glm::vec3 force(0, 0, 0);

    float Vnormal = glm::dot(velocity, normal);
    float Fnormal = glm::dot(node->Forces, normal);

    // if we are inside the fluid (solid ground is below us)
    if (gm->solid_ground_level != 0.0f && penetration >= 0)
    {
        // TODO:
        float Vsquared = glm::length2(velocity);

        // First of all calculate power law fluid viscosity
        float m = gm->flow_consistency_index * approx_pow(Vsquared, (gm->flow_behavior_index - 1.0f) * 0.5f);
        
        // Then calculate drag based on above. We'are using a simplified Stokes' drag.
        // Per node fluid drag surface coefficient set by node property applies here

        glm::vec3 Fdrag = velocity * (-m * node->surface_coef);

        // If we have anisotropic drag
        if (gm->drag_anisotropy < 1.0f && Vnormal > 0)
        {
            float da_factor;
            if (Vsquared > gm->va * gm->va)
                da_factor = 1.0f;
            else
                da_factor = Vsquared / (gm->va * gm->va);
            Fdrag += (Vnormal * m * (1.0f - gm->drag_anisotropy) * da_factor) * normal;
        }
        force += Fdrag;

        // Now calculate upwards force based on a simplified boyancy equation;
        // If the fluid is pseudoplastic then boyancy is constrained to only "stopping" a node from going downwards
        // Buoyancy per node volume coefficient set by node property applies here
        float Fboyancy = gm->fluid_density * penetration * (-DEFAULT_GRAVITY) * node->volume_coef;
        if (gm->flow_behavior_index < 1.0f && Vnormal >= 0.0f)
        {
            if (Fnormal < 0 && Fboyancy > -Fnormal)
            {
                Fboyancy = -Fnormal;
            }
        }
        force += Fboyancy * normal;

    }

    // if we are inside or touching the solid ground
    if (penetration >= gm->solid_ground_level)
    {
        // steady force
        float Freaction = -Fnormal;

        //impact force
        if (Vnormal < 0)
        {
            float penetration_depth = gm->solid_ground_level - penetration;
            Freaction -= (0.8f * Vnormal + 0.2f * penetration_depth / dt) * mass / dt; // Newton's second law
        }
        if (Freaction > 0)
        {
            glm::vec3 slipf = node->Forces - Fnormal * normal;
            glm::vec3 slip = velocity - Vnormal * normal;
            float slipv = glm::length(slip);
            slip = glm::normalize(slip);
            // If the velocity that we slip is lower than adhesion velocity and
            // we have a downforce and the slip forces are lower than static friction
            // forces then it's time to go into static friction physics mode.
            // This code is a direct translation of textbook static friction physics
            float Greaction = Freaction * gm->strength * node->friction_coef; //General moderated reaction
            float msGreaction = gm->ms * Greaction;
            if (slipv < gm->va && Greaction > 0.0f && glm::length2(slipf) <= msGreaction * msGreaction) 
            {
                // Static friction model (with a little smoothing to help the integrator deal with it)
                float ff = -msGreaction * (1.0f - approx_exp(-slipv / gm->va));
                force += Freaction * normal + ff * slip - slipf;
            }
            else
            {
                // Stribek model. It also comes directly from textbooks.
                float g = gm->mc + (gm->ms - gm->mc) * approx_exp(-approx_pow(slipv / gm->vs, gm->alpha));
                float ff = -(g + std::min(gm->t2 * slipv, 5.0f)) * Greaction;
                force += Freaction * normal + ff * slip;
            }
            node->nd_avg_collision_slip = node->nd_avg_collision_slip * 0.995f + slipv * 0.005f;
            node->nd_last_collision_slip = slipv * slip;
            node->nd_last_collision_force = std::min(-Freaction, 0.0f) * normal;
        }
    }

    return force;
}

int Collisions::createCollisionDebugVisualization()
{
    return 0;
}

int Collisions::addCollisionMesh(std::string meshname, glm::vec3 pos, glm::quat q, glm::vec3 scale, ground_model_t* gm, std::vector<int>* collTris)
{
    return 0;
}

void Collisions::getMeshInformation(
    void* mesh, size_t& vertex_count, glm::vec3*& vertices,
    size_t& index_count, unsigned*& indices,
    const glm::vec3& position,
    const glm::quat& orient, const glm::vec3& scale)
{

}

void Collisions::finishLoadingTerrain()
{

}




