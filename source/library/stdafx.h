#pragma once

#pragma execution_character_set("utf-8")

#define NOMINMAX

#include <cstdio>
#include <memory>
#include <mutex>
#include <vector>
#include <string>
#include <map>
#include <list>
#include <utility>
#include <cstdlib>
#include <ostream>
#include <sstream>
#include <array>
#include <unordered_map>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtc/type_ptr.hpp>

//#include <boost/stacktrace.hpp>


#if defined(WIN32) || defined(WIN64)
#include <Windows.h>
#define WIN32_LEAN_AND_MEAN
#endif