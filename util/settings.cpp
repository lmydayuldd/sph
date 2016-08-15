#include "util/settings.h"

// Graphics:

unsigned int Settings::SPHERE_DETAIL = 23;
bool         Settings::PAINT_VECTORS = true;
float        Settings::VECTOR_LENGTH_MULTIPLIER = 10.;

// Simulation:

float        Settings::dt              = 0.04;
float        Settings::WAVES_VELOCITY  = 4.0;
float        Settings::WAVES_DT        = 0.05;
double       Settings::ARENA_DIAMETER  = 20;
unsigned int Settings::PARTICLE_COUNT  = 400;
bool         Settings::PARTICLES_REACT = true;

// Physics:

float Settings::PARTICLE_MASS   = 10.; // 10.
float Settings::PARTICLE_RADIUS = 0.25; // 0.05

unsigned int Settings::SPH_NEIGHBOUR_COUNT = 5;
double Settings::SPH_SMOOTHING_LENGTH      = 2 * PARTICLE_RADIUS;
double Settings::SPH_MESH_CELL_DIAMETER    = PARTICLE_RADIUS; // to low caused a crash :p?...
double Settings::SPH_STIFFNESS_CONSTANT    = 300.; // m/s
double Settings::SPH_DESIRED_REST_DENSITY  = 1000.;
double Settings::SPH_VISCOSITY             = 0.01; // 10^-6 but large values preffered for simulation
double Settings::SPH_PRESSURE              = 0.;
double Settings::SPH_TEMPERATURE           = 23.;

bool   Settings::FORCES_EARTH_GRAVITY     = true;
bool   Settings::FORCES_UNIVERSAL_GRAVITY = false;
bool   Settings::FORCES_COULOMB           = false;
double Settings::FORCES_HOOKE_SPRING      = 0.7;
float  Settings::FORCES_HOOKE_DISTANCE    = 0.3;
double Settings::FORCES_HOOKE_DAMP        = 0.;
float  Settings::FORCES_FRICTION          = 0.01;
float  Settings::DAMPENING                = 0.0; // 1.0 - ELASTICITY
