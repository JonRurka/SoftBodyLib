#pragma once

#include <stdafx.h>

#if defined(_WIN32) || defined(_WIN64)

#define EXPORTED extern "C" __declspec(dllexport)

#elif defined(__linux__)

#define EXPORTED extern "C"

#else

#error Unsupported platform

#endif

struct C_Vec2 {
public:
    float x{ 0 };
    float y{ 0 };

    static C_Vec2 To(glm::vec2 o);

    static glm::vec2 From(C_Vec2 o);
};

struct C_Vec3 {
public:
    float x{ 0 };
    float y{ 0 };
    float z{ 0 };

    static C_Vec3 To(glm::vec3 o);

    static glm::vec3 From(C_Vec3 o);
};

struct C_Vec4 {
public:
    float x{ 0 };
    float y{ 0 };
    float z{ 0 };
    float w{ 0 };

    static C_Vec4 To(glm::vec4 o);

    static glm::vec4 From(C_Vec4 o);
};

struct C_Quat {
public:
    float x{ 0 };
    float y{ 0 };
    float z{ 0 };
    float w{ 0 };

    static C_Quat To(glm::quat o);

    static glm::quat From(C_Quat o);
};

