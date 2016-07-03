#include "util/settings.h"

// Graphics:

unsigned int Settings::SPHERE_DETAIL = 23;
bool         Settings::PAINT_VECTORS = true;
float        Settings::VECTOR_LENGTH_MULTIPLIER = 10.;

// Physics:

unsigned int Settings::PARTICLE_COUNT          = 13;
bool         Settings::PARTICLES_REACT         = true;
bool         Settings::DIMENSIONS_3D           = false;
bool         Settings::PARTICLES_INITIAL_SPEED = false;

float Settings::PARTICLE_MASS   = 10.;
float Settings::PARTICLE_RADIUS = 0.05;

bool   Settings::FORCES_EARTH_GRAVITY     = true;
bool   Settings::FORCES_UNIVERSAL_GRAVITY = false;
bool   Settings::FORCES_COULOMB           = false;
double Settings::FORCES_HOOKE_SPRING   = 0.7;
float  Settings::FORCES_HOOKE_DISTANCE = 0.3;
double Settings::FORCES_HOOKE_DAMP     = 0.;
float  Settings::FORCES_FRICTION = 0.01;
float  Settings::DAMPENING       = 0.00; // 1.0 - ELASTICITY

float  Settings::dt = 0.01;
float  Settings::WAVES_VELOCITY = 4.0;
float  Settings::WAVES_DT       = 0.05;
