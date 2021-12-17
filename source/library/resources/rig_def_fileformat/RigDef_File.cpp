#include "RigDef_File.h"


const char* ROOT_MODULE_NAME = "_Root_"; // Static

NodeDefaults::NodeDefaults() :
    load_weight(-1.f),
    friction(1),
    volume(1),
    surface(1),
    options(0)
{}

File::File() :
    file_format_version(0), // Default = unset
    enable_advanced_deformation(false)
{
    root_module = std::make_shared<File::Module>(ROOT_MODULE_NAME); // Required to exist.
}

File::Module::Module(std::string const& name) :
    name(name)
{
    beams.reserve(1000);
    nodes.reserve(2000);
}

const char* File::SubsectionToString(File::Subsection subsection)
{
    switch (subsection)
    {
    case (File::SUBSECTION_NONE):
        return "SUBSECTION_NONE";
    case (File::SUBSECTION__FLEXBODIES__PROPLIKE_LINE):
        return "SUBSECTION_PROPLIKE_LINE";
    case (File::SUBSECTION__FLEXBODIES__FORSET_LINE):
        return "SUBSECTION_FORSET_LINE";
    case (File::SUBSECTION__SUBMESH__TEXCOORDS):
        return "SUBSECTION_TEXCOORDS";
    case (File::SUBSECTION__SUBMESH__CAB):
        return "SUBSECTION_CAB";
    default:
        return "~ UNKNOWN SUBSECTION ~";
    }
}

const char* File::SectionToString(File::Section section)
{
    switch (section)
    {
    case (File::SECTION_AIRBRAKES):
        return "airbrakes";
    case (File::SECTION_ANIMATORS):
        return "animators";
    case (File::SECTION_ANTI_LOCK_BRAKES):
        return "AntiLockBrakes";
    case (File::SECTION_AUTHOR):
        return "author";
    case (File::SECTION_AXLES):
        return "axles";
    case (File::SECTION_BEAMS):
        return "beams";
    case (File::SECTION_BRAKES):
        return "brakes";
    case (File::SECTION_CAMERAS):
        return "cameras";
    case (File::SECTION_CAMERA_RAIL):
        return "camerarail";
    case (File::SECTION_CINECAM):
        return "cinecam";
    case (File::SECTION_COLLISION_BOXES):
        return "collisionboxes";
    case (File::SECTION_COMMANDS):
        return "commands";
    case (File::SECTION_COMMANDS_2):
        return "commands2";
    case (File::SECTION_CONTACTERS):
        return "contacters";
    case (File::SECTION_ENGINE):
        return "engine";
    case (File::SECTION_ENGOPTION):
        return "engoption";
    case (File::SECTION_ENGTURBO):
        return "engturbo";
    case (File::SECTION_EXHAUSTS):
        return "exhausts";
    case (File::SECTION_FIXES):
        return "fixes";
    case (File::SECTION_FLARES):
        return "flares";
    case (File::SECTION_FLARES_2):
        return "flares2";
    case (File::SECTION_FLEXBODIES):
        return "flexbodies";
    case (File::SECTION_FLEX_BODY_WHEELS):
        return "flexbodywheels";
    case (File::SECTION_FUSEDRAG):
        return "fusedrag";
    case (File::SECTION_GLOBALS):
        return "globals";
    case (File::SECTION_GUI_SETTINGS):
        return "guisettings";
    case (File::SECTION_HYDROS):
        return "hydros";
    case (File::SECTION_HELP):
        return "help";
    case (File::SECTION_HOOKS):
        return "hooks";
    case (File::SECTION_INTERAXLES):
        return "interaxles";
    case (File::SECTION_LOCKGROUPS):
        return "lockgroups";
    case (File::SECTION_MANAGED_MATERIALS):
        return "managedmaterials";
    case (File::SECTION_MAT_FLARE_BINDINGS):
        return "materialflarebindings";
    case (File::SECTION_MESH_WHEELS):
        return "meshwheels";
    case (File::SECTION_MESH_WHEELS_2):
        return "meshwheels2";
    case (File::SECTION_MINIMASS):
        return "minimass";
    case (File::SECTION_NODES):
        return "nodes";
    case (File::SECTION_NODES_2):
        return "nodes2";
    case (File::SECTION_PARTICLES):
        return "particles";
    case (File::SECTION_PISTONPROPS):
        return "pistonprops";
    case (File::SECTION_PROPS):
        return "props";
    case (File::SECTION_RAILGROUPS):
        return "railgroups";
    case (File::SECTION_ROPABLES):
        return "ropables";
    case (File::SECTION_ROPES):
        return "ropes";
    case (File::SECTION_ROTATORS):
        return "rotators";
    case (File::SECTION_ROTATORS_2):
        return "rotators_2";
    case (File::SECTION_SCREWPROPS):
        return "screwprops";
    case (File::SECTION_SHOCKS):
        return "shocks";
    case (File::SECTION_SHOCKS_2):
        return "shocks2";
    case (File::SECTION_SHOCKS_3):
        return "shocks3";
    case (File::SECTION_SLIDENODES):
        return "slidenodes";
    case (File::SECTION_SOUNDSOURCES):
        return "soundsources";
    case (File::SECTION_SOUNDSOURCES2):
        return "soundsources2";
    case (File::SECTION_SLOPE_BRAKE):
        return "SlopeBrake";
    case (File::SECTION_SUBMESH):
        return "submesh";
    case (File::SECTION_TIES):
        return "ties";
    case (File::SECTION_TORQUE_CURVE):
        return "torquecurve";
    case (File::SECTION_TRACTION_CONTROL):
        return "TractionControl";
    case (File::SECTION_TRANSFER_CASE):
        return "transfercase";
    case (File::SECTION_TRIGGERS):
        return "triggers";
    case (File::SECTION_TRUCK_NAME):
        return "<truck name>";
    case (File::SECTION_TURBOJETS):
        return "turbojets";
    case (File::SECTION_TURBOPROPS):
        return "turboprops";
    case (File::SECTION_TURBOPROPS_2):
        return "turboprops";
    case (File::SECTION_VIDEO_CAMERA):
        return "videocamera";
    case (File::SECTION_WHEELS):
        return "wheels";
    case (File::SECTION_WHEELS_2):
        return "wheels2";
    case (File::SECTION_WINGS):
        return "wings";

    case (File::SECTION_NONE):
        return "SECTION_NONE";
    default:
        return "<unknown>";
    }
}

const char* File::KeywordToString(File::Keyword keyword)
{
    /* NOTE: Maintain alphabetical order! */

    switch (keyword)
    {
    case (File::KEYWORD_AIRBRAKES):
        return "airbrakes";
    case (File::KEYWORD_ANIMATORS):
        return "animators";
    case (File::KEYWORD_ANTI_LOCK_BRAKES):
        return "AntiLockBrakes";
    case (File::KEYWORD_AUTHOR):
        return "author";
    case (File::KEYWORD_AXLES):
        return "axles";
    case (File::KEYWORD_BEAMS):
        return "beams";
    case (File::KEYWORD_BRAKES):
        return "brakes";
    case (File::KEYWORD_CAB):
        return "submesh >> cab";
    case (File::KEYWORD_CAMERAS):
        return "cameras";
    case (File::KEYWORD_CAMERARAIL):
        return "camerarail";
    case (File::KEYWORD_CINECAM):
        return "cinecam";
    case (File::KEYWORD_COLLISIONBOXES):
        return "collisionboxes";
    case (File::KEYWORD_COMMANDS):
        return "commands";
    case (File::KEYWORD_COMMANDS2):
        return "commands2";
    case (File::KEYWORD_CONTACTERS):
        return "contacters";
    case (File::KEYWORD_CRUISECONTROL):
        return "cruisecontrol";
    case (File::KEYWORD_DESCRIPTION):
        return "description";
    case (File::KEYWORD_ENGINE):
        return "engine";
    case (File::KEYWORD_ENGOPTION):
        return "engoption";
    case (File::KEYWORD_ENGTURBO):
        return "engturbo";
    case (File::KEYWORD_EXHAUSTS):
        return "exhausts";
    case (File::KEYWORD_FILEINFO):
        return "fileinfo";
    case (File::KEYWORD_FILEFORMATVERSION):
        return "fileformatversion";
    case (File::KEYWORD_FIXES):
        return "fixes";
    case (File::KEYWORD_FLARES):
        return "flares";
    case (File::KEYWORD_FLARES2):
        return "flares2";
    case (File::KEYWORD_FLEXBODIES):
        return "flexbodies";
    case (File::KEYWORD_FLEXBODYWHEELS):
        return "flexbodywheels";
    case (File::KEYWORD_FUSEDRAG):
        return "fusedrag";
    case (File::KEYWORD_GLOBALS):
        return "globals";
    case (File::KEYWORD_GUISETTINGS):
        return "guisettings";
    case (File::KEYWORD_HELP):
        return "help";
    case (File::KEYWORD_HOOKS):
        return "hooks";
    case (File::KEYWORD_HYDROS):
        return "hydros";
    case (File::KEYWORD_INTERAXLES):
        return "interaxles";
    case (File::KEYWORD_MANAGEDMATERIALS):
        return "managedmaterials";
    case (File::KEYWORD_MATERIALFLAREBINDINGS):
        return "materialflarebindings";
    case (File::KEYWORD_MESHWHEELS):
        return "meshwheels";
    case (File::KEYWORD_MESHWHEELS2):
        return "meshwheels2";
    case (File::KEYWORD_MINIMASS):
        return "minimass";
    case (File::KEYWORD_NODES):
        return "nodes";
    case (File::KEYWORD_NODES2):
        return "nodes2";
    case (File::KEYWORD_PARTICLES):
        return "particles";
    case (File::KEYWORD_PISTONPROPS):
        return "pistonprops";
    case (File::KEYWORD_PROPS):
        return "props";
    case (File::KEYWORD_RAILGROUPS):
        return "railgroups";
    case (File::KEYWORD_ROPABLES):
        return "ropables";
    case (File::KEYWORD_ROPES):
        return "ropes";
    case (File::KEYWORD_ROTATORS):
        return "rotators";
    case (File::KEYWORD_ROTATORS2):
        return "rotators_2";
    case (File::KEYWORD_SCREWPROPS):
        return "screwprops";
    case (File::KEYWORD_SHOCKS):
        return "shocks";
    case (File::KEYWORD_SHOCKS2):
        return "shocks2";
    case (File::KEYWORD_SHOCKS3):
        return "shocks3";
    case (File::KEYWORD_SLIDENODES):
        return "slidenodes";
    case (File::KEYWORD_SLOPE_BRAKE):
        return "SlopeBrake";
    case (File::KEYWORD_SOUNDSOURCES):
        return "soundsources";
    case (File::KEYWORD_SOUNDSOURCES2):
        return "soundsources2";
    case (File::KEYWORD_SUBMESH):
        return "submesh";
    case (File::KEYWORD_TEXCOORDS):
        return "submesh >> texcoords";
    case (File::KEYWORD_TIES):
        return "ties";
    case (File::KEYWORD_TORQUECURVE):
        return "torquecurve";
    case (File::KEYWORD_TRACTION_CONTROL):
        return "TractionControl";
    case (File::KEYWORD_TRANSFER_CASE):
        return "transfercase";
    case (File::KEYWORD_TRIGGERS):
        return "triggers";
    case (File::KEYWORD_TURBOJETS):
        return "turbojets";
    case (File::KEYWORD_TURBOPROPS):
        return "turboprops";
    case (File::KEYWORD_TURBOPROPS2):
        return "turboprops2";
    case (File::KEYWORD_VIDEOCAMERA):
        return "videocamera";
    case (File::KEYWORD_WHEELS):
        return "wheels";
    case (File::KEYWORD_WHEELS2):
        return "wheels2";
    case (File::KEYWORD_WINGS):
        return "wings";

    default:
        return "~Unknown~";
    }
}