#pragma once

#include "Physics.h"
#include "resources/resources.h"
#include "terrain/terrain.h"


namespace SoftBodyLib {
	class ActorSpawner
	{


    public:
        struct ActorMemoryRequirements
        {
            ActorMemoryRequirements() { memset(this, 0, sizeof(ActorMemoryRequirements)); }

            size_t num_nodes;
            size_t num_beams;
            size_t num_fixes;
            // ... more to come ...
        };


        void Setup(
            Actor* actor,
            std::shared_ptr<File> file,
            glm::vec3 const& spawn_position
        );

        Actor* SpawnActor();

        /**
        * Adds a vehicle module to the validated configuration.
        * @param module_name A module from the validated rig-def file.
        */
        void AddModule(std::shared_ptr<File::Module> module)
        {
            m_selected_modules.push_back(module);
        }

        Actor* GetActor()
        {
            return m_actor;
        }

        ActorMemoryRequirements const& GetMemoryRequirements()
        {
            return m_memory_requirements;
        }

        /**
        * Finds existing node by Node::Ref; throws an exception if the node doesn't exist.
        * @return Index of existing node
        * @throws Exception If the node isn't found.
        */
        unsigned int GetNodeIndexOrThrow(Node::Ref const& id);

    private:

        void InitializeRig();

        void CalcMemoryRequirements(ActorMemoryRequirements& req, File::Module* module_def);

        /**
        * Section 'gobals' in any module
        */
        void ProcessGlobals(Globals& def);

        void ProcessNode(Node& def);

        /**
        * Section 'beams'. Depends on 'nodes'
        */
        void ProcessBeam(Beam& def);

        /**
        * Section 'submeshes'.
        */
        void ProcessSubmesh(Submesh& def);

        /**
        * Section 'contacters'.
        */
        void ProcessContacter(Node::Ref& node_ref);

        /**
        * Section 'collisionboxes'
        */
        void ProcessCollisionBox(CollisionBox& def);
        
        /**
        * Section 'flexbodies'.
        */
        void ProcessFlexbody(std::shared_ptr<Flexbody> def);


        void FinalizeRig();


        /**
        * Checks there is still space left in rig_t::subtexcoords, rig_t::subcabs and rig_t::subisback arrays.
        * @param count Required number of free slots.
        * @return True if there is space left.
        */
        bool CheckSubmeshLimit(unsigned int count);

        /**
        * Checks there is still space left in rig_t::texcoords array.
        * @param count Required number of free slots.
        * @return True if there is space left.
        */
        bool CheckTexcoordLimit(unsigned int count);

        /**
        * Checks there is still space left in rig_t::cabs array.
        * @param count Required number of free slots.
        * @return True if there is space left.
        */
        bool CheckCabLimit(unsigned int count);

        /**
        * Finds existing node by Node::Ref
        * @return First: Index of existing node; Second: true if node was found.
        */
        std::pair<unsigned int, bool> GetNodeIndex(Node::Ref const& node_ref, bool quiet = false);

        /**
        * Adds a node to the rig.
        * @return First: node index, second: True if the node was inserted, false if duplicate.
        */
        std::pair<unsigned int, bool> AddNode(Node::Id& id);

        /**
        * Seeks node.
        * @return Pointer to node, or nullptr if not found.
        */
        node_t* GetBeamNodePointer(Node::Ref const& node_ref);

        /**
        * Finds existing node by Node::Ref
        * @return Pointer to node or nullptr if not found.
        */
        node_t* GetNodePointer(Node::Ref const& node_ref);

        /**
        * Gets a free beam slot; checks limits, sets it's array position and updates 'free_beam' index.
        * @return A reference to beam slot.
        */
        beam_t& GetFreeBeam();

        /**
        * Gets a free beam slot; checks limits, sets it's array position and updates 'rig_t::free_beam' index.
        * @return A reference to beam slot.
        */
        beam_t& GetAndInitFreeBeam(node_t& node_1, node_t& node_2);

        /**
        * Fetches free beam and sets up defaults.
        */
        beam_t& AddBeam(
            node_t& node_1,
            node_t& node_2,
            std::shared_ptr<BeamDefaults>& defaults,
            int detacher_group
        );

        void CalculateBeamLength(beam_t& beam);

        void SetBeamDeformationThreshold(beam_t& beam, std::shared_ptr<BeamDefaults> beam_defaults);

        void UpdateCollcabContacterNodes();

        Actor* m_actor; //!< The output actor.
        glm::vec3      m_spawn_position;

        std::vector<CabTexcoord>  m_oldstyle_cab_texcoords;
        std::vector<CabSubmesh>   m_oldstyle_cab_submeshes;
        ActorMemoryRequirements   m_memory_requirements;
        std::shared_ptr<File>     m_file; //!< The parsed input file.
        std::map<std::string, unsigned int>   m_named_nodes;

        std::list<std::shared_ptr<File::Module>>  m_selected_modules;
	};
}
