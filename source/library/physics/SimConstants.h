#pragma once

constexpr int MAX_SUBMESHES = 500;             //!< maximum number of submeshes per actor
constexpr int MAX_TEXCOORDS = 3000;            //!< maximum number of texture coordinates per actor

constexpr float DEFAULT_GRAVITY = -9.807f;       //!< earth gravity
constexpr float DEFAULT_DRAG = 0.05f;


constexpr float DEFAULT_SPRING = 9000000.0f;
constexpr float DEFAULT_DAMP = 12000.0f;
constexpr float DEFAULT_BEAM_DIAMETER = 0.05f;
constexpr float MIN_BEAM_LENGTH = 0.1f;          //!< minimum beam lenght is 10 centimeters
constexpr float INVERTED_MIN_BEAM_LENGTH = 1.0f / MIN_BEAM_LENGTH;
constexpr float BEAM_SKELETON_DIAMETER = 0.01f;
constexpr float BEAM_BREAK = 1000000.0f;
constexpr float BEAM_DEFORM = 400000.0f;
constexpr float BEAM_CREAK_DEFAULT = 100000.0f;

constexpr int   MAX_CABS = 3000;            //!< maximum number of cabs per actor

constexpr float DEFAULT_COLLISION_RANGE = 0.02f;


constexpr float DEFAULT_MINIMASS = 50.0;


