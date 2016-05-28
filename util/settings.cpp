#include "util/settings.h"

unsigned int Settings::PARTICLE_COUNT  = 13;    // used by Particle
unsigned int Settings::SPHERE_DETAIL   = 23;    // used by Particle, CollisionSphere
bool         Settings::PAINT_VECTORS   = true;  // used by Vector
bool         Settings::PARTICLES_REACT = true; // used by Particle, ?glRenderer?
bool         Settings::DIMENSIONS_3D   = false;

float  Settings::dt = 0.01;
float  Settings::DAMPENING = 0.00; // 1.0 - ELASTICITY

bool   Settings::FORCES_EARTH_GRAVITY     = true;
bool   Settings::FORCES_UNIVERSAL_GRAVITY = true;
bool   Settings::FORCES_COULOMB           = false;

double Settings::FORCES_HOOKE_SPRING   = 0.7;
float  Settings::FORCES_HOOKE_DISTANCE = 0.3f;
double Settings::FORCES_HOOKE_DAMP     = 0.;

float  Settings::FORCES_FRICTION = 0.01f;

float  Settings::WAVES_VELOCITY = 4.0f;
float  Settings::WAVES_DT       = 0.05f;
