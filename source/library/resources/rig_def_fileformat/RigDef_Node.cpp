#include "RigDef_Node.h"
#include "RigDef_File.h"

Node::Id::Id():
	m_id_num(0),
	m_flags(0)
{}

Node::Id::Id(unsigned int num) :
    m_id_num(num),
    m_flags(0)
{
    this->SetNum(num);
}

Node::Id::Id(std::string const& id_str) :
    m_id_num(0),
    m_flags(0)
{
    this->setStr(id_str);
}

void Node::Id::SetNum(unsigned int num)
{
    m_id_num = num;
    m_id_str = std::to_string(num);
    BITMASK_SET_0(m_flags, IS_TYPE_NAMED);
    BITMASK_SET_1(m_flags, IS_TYPE_NUMBERED | IS_VALID);
}

void Node::Id::setStr(std::string const& id_str)
{
    m_id_num = 0;
    m_id_str = id_str;
    BITMASK_SET_0(m_flags, IS_TYPE_NUMBERED);
    BITMASK_SET_1(m_flags, IS_TYPE_NAMED | IS_VALID);
}

void Node::Id::Invalidate()
{
    m_id_num = 0;
    m_id_str.clear();
    m_flags = 0;
}

std::string Node::Id::ToString() const
{
    return std::string("Node::Id(") + this->Str() + (this->IsTypeNumbered() ? " NUMBERED)" : " NAMED)");
}

Node::Ref::Ref(std::string const& id_str, unsigned int id_num, unsigned flags, unsigned line_number):
    m_flags(0),
    m_line_number(line_number),
    m_id_as_number(id_num)
{
    m_id = id_str;
    BITMASK_SET_1(m_flags, flags);
}

Node::Ref::Ref() :
    m_flags(0),
    m_id_as_number(0),
    m_line_number(0)
{
}

void Node::Ref::Invalidate()
{
    m_id_as_number = 0;
    m_id.clear();
    m_flags = 0;
    m_line_number = 0;
}

std::string Node::Ref::ToString() const
{
    std::stringstream msg;
    msg << "Node::Ref(id:" << m_id
        << ", src line:";
    if (m_line_number != 0)
    {
        msg << m_line_number;
    }
    else
    {
        msg << "?";
    }
    msg << ", import flags:[";
    if (GetImportState_IsValid()) { msg << " VALID"; }
    if (GetImportState_MustCheckNamedFirst()) { msg << " CHECK_NAMED_FIRST"; }
    if (GetImportState_IsResolvedNamed()) { msg << " RESOLVED_NAMED"; }
    if (GetImportState_IsResolvedNumbered()) { msg << " RESOLVED_NUMBERED"; }
    msg << "], regular flags:[";
    if (GetRegularState_IsValid()) { msg << " VALID"; }
    if (GetRegularState_IsNamed()) { msg << " NAMED"; }
    if (GetRegularState_IsNumbered()) { msg << " NUMBERED"; }
    msg << "])";
    return msg.str();
}




