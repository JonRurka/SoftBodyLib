#pragma once

//#include "../../utils/BitFlags.h"
//#include <string>
//#include <memory>
//#include <vector>
//#include <sstream>
//#include <glm/glm.hpp>

//#include "rigdef.h"
#include "stdafx.h"
#include "utils/BitFlags.h"
#include "RigDef_Prerequisites.h"
//#include "RigDef_File.h"


struct Node
{
    /// Abstract node ID (numbered or named)
    /// Node name is always available. For numbered nodes, it's the number converted to string.
    /// Node number is only available for explicitly numbered nodes (legacy).
    class Id
    {
    public:
        BITMASK_PROPERTY_GET(m_flags, 1, IS_VALID, IsValid);
        BITMASK_PROPERTY_GET(m_flags, 2, IS_TYPE_NUMBERED, IsTypeNumbered);
        BITMASK_PROPERTY_GET(m_flags, 3, IS_TYPE_NAMED, IsTypeNamed);

        // Constructors
        Id();
        explicit Id(unsigned int id_num);
        explicit Id(std::string const& id_str);

        // Setters
        void SetNum(unsigned int id_num);
        void setStr(std::string const& id_str);

        // Getters
        inline std::string const& Str() const { return m_id_str; }
        inline unsigned int       Num() const { return m_id_num; }

        // Util
        void Invalidate();
        std::string ToString() const;


    private:
        unsigned int m_id_num;
        std::string  m_id_str;
        unsigned int m_flags;
    };

    class Ref
    {
    public:
        // Since fileformatversion is not known from the beginning of parsing, 2 states must be kept 
        // at the same time: IMPORT_STATE and REGULAR_STATE. The outer logic must make the right pick.
        BITMASK_PROPERTY(m_flags, 1, IMPORT_STATE_IS_VALID, GetImportState_IsValid, SetImportState_IsValid);
        BITMASK_PROPERTY_GET(m_flags, 2, IMPORT_STATE_MUST_CHECK_NAMED_FIRST, GetImportState_MustCheckNamedFirst);
        BITMASK_PROPERTY_GET(m_flags, 3, IMPORT_STATE_IS_RESOLVED_NAMED, GetImportState_IsResolvedNamed);
        BITMASK_PROPERTY_GET(m_flags, 4, IMPORT_STATE_IS_RESOLVED_NUMBERED, GetImportState_IsResolvedNumbered);

        BITMASK_PROPERTY(m_flags, 5, REGULAR_STATE_IS_VALID, GetRegularState_IsValid, SetRegularState_IsValid);
        BITMASK_PROPERTY_GET(m_flags, 6, REGULAR_STATE_IS_NAMED, GetRegularState_IsNamed);
        BITMASK_PROPERTY_GET(m_flags, 7, REGULAR_STATE_IS_NUMBERED, GetRegularState_IsNumbered);

        Ref(std::string const& id_str, unsigned int id_num, unsigned flags, unsigned line_number_defined);
        Ref();

        inline std::string const& Str() const { return m_id; }
        inline unsigned int       Num() const { return m_id_as_number; }

        inline bool Compare(Ref const& rhs) const { return m_id == rhs.m_id; }
        inline bool operator==(Ref const& rhs) const { return Compare(rhs); }
        inline bool operator!=(Ref const& rhs) const { return !Compare(rhs); }

        inline bool     IsValidAnyState() const { return GetImportState_IsValid() || GetRegularState_IsValid(); }
        inline unsigned GetLineNumber() const { return m_line_number; }

        void Invalidate();
        std::string ToString() const;


    private:
        std::string  m_id;
        unsigned int m_id_as_number;
        unsigned int m_flags;
        unsigned int m_line_number;
    };

    struct Range
    {
        Range(Node::Ref const& start, Node::Ref const& end) :
            start(start),
            end(end)
        {}

        explicit Range(Node::Ref const& single) :
            start(single),
            end(single)
        {}

        inline bool IsRange() const { return start != end; }

        void SetSingle(Node::Ref const& ref)
        {
            start = ref;
            end = ref;
        }

        Node::Ref start;
        Node::Ref end;
    };

    Id id;
    glm::vec3 position;
    unsigned int options; //!< Bit flags
    float load_weight_override;
    bool has_load_weight_override;
    std::shared_ptr<NodeDefaults> node_defaults;
    std::shared_ptr<MinimassPreset> node_minimass;
    std::shared_ptr<BeamDefaults> beam_defaults; /* Needed for hook */
    int detacher_group;
};
