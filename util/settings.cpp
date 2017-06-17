#include "util/settings.h"

#include <limits>
#include <cmath>

using namespace std;

// Graphics:

bool         Settings::DOT_OR_SPHERE            = true;
unsigned int Settings::SPHERE_DETAIL            = 23;
bool         Settings::PAINT_VECTORS            = true;
bool         Settings::COLOR_BY_SPEED           = false;
bool         Settings::COLOR_BOUNDARIES         = false;
double       Settings::VECTOR_LENGTH_MULTIPLIER = 10.;

// Simulation:

float        Settings::WAVES_VELOCITY  = 4.0;
float        Settings::WAVES_DT        = 0.05;

double       Settings::dt              = 0.04 * 3; // 0.12
double       Settings::ARENA_DIAMETER  = 20; // 20
unsigned int Settings::PARTICLE_COUNT  = 800; // 400
bool         Settings::PARTICLES_REACT = true;

// Physics:

double Settings::PARTICLE_RADIUS = 0.15; // 0.25
double Settings::GRAVITY_VALUE   = -9.81; // for SPH

bool Settings::SPH_NEIGHBOUR_BY_CNT_OR_DIST = true;
unsigned int Settings::SPH_NEIGHBOUR_COUNT  = 10000;
unsigned int Settings::SPH_KERNEL_DIM       = 3;
double Settings::SPH_PARTICLES_INIT_DIST    = PARTICLE_RADIUS * 1.2;//2.0;
double Settings::SPH_PARTICLE_MAX_DR        = PARTICLE_RADIUS * 1000.4;//0.4;
double Settings::SPH_MESH_CELL_DIAMETER     = PARTICLE_RADIUS * 4; // 8 // to low caused a crash :p?...
//double Settings::SPH_MESH_CELL_DIAMETER     = ARENA_DIAMETER; // 8 // to low caused a crash :p?...
double Settings::SPH_SMOOTHING_LENGTH       = PARTICLE_RADIUS * 4;
double Settings::SPH_NEIGHBOUR_RANGE        = PARTICLE_RADIUS * 4;//std::numeric_limits<double>::infinity();//2 * smoothing_length;
double Settings::SPH_STIFFNESS_CONSTANT     = 300.; // 300. // m/s
double Settings::SPH_DESIRED_REST_DENSITY   = 1000.;
double Settings::SPH_VISCOSITY              = 0.01; // 0.01 // 10^-6 but large values preffered for simulation

double Settings::PARTICLE_MASS   = 10.; // 10.
//double Settings::PARTICLE_MASS   = SPH_DESIRED_REST_DENSITY * pow(Settings::SPH_SMOOTHING_LENGTH, 3);
//double Settings::PARTICLE_MASS   = SPH_DESIRED_REST_DENSITY * pow(2./3 * Settings::SPH_SMOOTHING_LENGTH, 3);

double Settings::SPH_PRESSURE               = 0.;
double Settings::SPH_TEMPERATURE            = 23.;

bool   Settings::FORCES_EARTH_GRAVITY     = true;
bool   Settings::FORCES_UNIVERSAL_GRAVITY = false;
bool   Settings::FORCES_COULOMB           = false;
double Settings::WALL_DAMPENING           = 0.00; // 1.0 - ELASTICITY
double Settings::WATER_DAMPENING          = 0.00;
double Settings::FORCES_FRICTION          = 0.01;
double Settings::FORCES_HOOKE_SPRING      = 0.7;
float  Settings::FORCES_HOOKE_DISTANCE    = 0.3;
double Settings::FORCES_HOOKE_DAMP        = 0.;

// Controls:

bool Settings::DRAG_OR_FORCE = true;
