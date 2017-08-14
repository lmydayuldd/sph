#include "util/enums.h"
#include "util/map.h"
#include "util/settings.h"
#include "control/interaction.h"

#include <limits>
#include <cmath>

using namespace std;

// Environment:

unsigned     Settings::WINDOW_WIDTH             = 720;
unsigned     Settings::WINDOW_HEIGHT            = 540;
bool         Settings::NO_PRINTOUT              = false;
bool         Settings::NO_SCREENS_NO_VIDEO      = false; // eliminates inconsistent framerate

// Graphics:

bool         Settings::DOT_OR_SPHERE            = true;
unsigned     Settings::SPHERE_DETAIL            = 23;
bool         Settings::PAINT_VECTORS            = true;
bool         Settings::COLOR_BY_SPEED           = false;
bool         Settings::COLOR_BOUNDARIES         = false;
double       Settings::VECTOR_LENGTH_MULTIPLIER = 30.; // 10.

// Controls:

unsigned char Settings::CONTROL_MODE = ONE_DRAG;

// Simulation:

double        Settings::dt                   = 0.16 * 3; // 0.48
unsigned      Settings::ITERATIONS_PER_FRAME = 1;
double        Settings::ARENA_DIAMETER       = 30.; // 20.
double        Settings::ARENA_DIAMETER_Z     = 0.;
unsigned      Settings::PARTICLE_COUNT       = 1800; // 1200
unsigned char Settings::MAP_SETUP            = DAM_BREAK_NON_MAP;
unsigned      Settings::GHOST_LAYER_GAGE     = 0;
bool          Settings::CALCULATE_MASS       = false; // TODO
bool          Settings::PARALLEL_GPU         = false;
bool          Settings::PARALLEL_MPI         = false;
bool          Settings::PARALLEL_OMP         = true;
unsigned      Settings::PARALLEL_OMP_THREADS = 8;
unsigned      Settings::PARTICLES_X          = 0;
unsigned      Settings::PARTICLES_Y          = 0;
unsigned      Settings::PARTICLES_Z          = 0;
double        Settings::WORLD_ROTATION       = 0.;

// Physics:

bool         Settings::PARTICLES_REACT      = true;
bool         Settings::COLLIDE_MOVE_OUT     = true;
bool         Settings::COLLIDE_TRANSFER_VEL = true;
bool         Settings::NEIGHBOUR_LMT_BY_CNT = true;
bool         Settings::NEIGHBOUR_LMT_BY_DST = true;
std::string  Settings::NEIGHBOUR_CHOICE     = "GRID"; // "ALL", "GRID", "OCTREE"
unsigned     Settings::NEIGHBOUR_COUNT      = 1000; // 5*5*5 - 1;
unsigned     Settings::KERNEL_DIM           = 3;
double       Settings::STIFFNESS_CONSTANT   = 300.; // 300. // m/s
double       Settings::DESIRED_REST_DENSITY = 1000.;
double       Settings::VISCOSITY            = 0.01; // 0.01 // 10^-6 but large values preffered for simulation
double       Settings::PARTICLE_RADIUS      = 0.15; // 0.25
double       Settings::PARTICLES_INIT_DIST  = PARTICLE_RADIUS * 2.0; // 2.0;
double       Settings::MESH_CELL_DIAMETER   = PARTICLE_RADIUS * 2;
                                            //= ARENA_DIAMETER;
double       Settings::SMOOTHING_LENGTH     = PARTICLE_RADIUS * 16; // smoothing length / kernel support
double       Settings::NEIGHBOUR_RANGE      = PARTICLE_RADIUS * 16; // 2 * smoothing_length; // std::numeric_limits<double>::infinity();
double       Settings::PARTICLE_MAX_DR      = PARTICLE_RADIUS * 1000.4; //0.4;
double       Settings::PARTICLE_MASS        = DESIRED_REST_DENSITY
//                                              * 10;
//                                              * pow(Settings::SMOOTHING_LENGTH, 3);
                                              * pow(2./3 * SMOOTHING_LENGTH, 3);

bool   Settings::FORCES_GRAVITY_EARTH     = false;
//double Settings::FORCES_GRAVITY_EARTH_VAL = 0.; // for SPH
double Settings::FORCES_GRAVITY_EARTH_VAL = -9.81; // for SPH
bool   Settings::FORCES_GRAVITY_UNIVERSAL = true;
bool   Settings::FORCES_COULOMB           = false;
double Settings::FORCES_DAMPEN_WALL       = 0.05; // 1.0 - ELASTICITY
double Settings::FORCES_DAMPEN_WATER      = 0.00;
double Settings::FORCES_FRICTION          = 0.00;

// LEGACY //////////////////////////////////////////////////////////////////////

double Settings::PRESSURE    = 0.;
double Settings::TEMPERATURE = 23.;

double Settings::FORCES_HOOKE_SPRING   = 0.7;
float  Settings::FORCES_HOOKE_DISTANCE = 0.3;
double Settings::FORCES_HOOKE_DAMP     = 0.;

float  Settings::WAVES_VELOCITY = 4.0;
float  Settings::WAVES_DT       = 0.05;
