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
#include <stdlib.h>
#include <ostream>
#include <array>

#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/norm.hpp>

/*#include <map>
#include <string>
#include <limits>
#include <cmath>
#include <algorithm>
#include <iostream>
#include <fstream>
#include <list>
#include <cstdarg>
#include <thread>*/


#ifdef WINDOWS_BUILD

#include <Windows.h>
#define WIN32_LEAN_AND_MEAN

#else 

#define LINUX_BUILD


#endif