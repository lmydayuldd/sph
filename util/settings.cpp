#include "util/enums.h"
#include "util/map.h"
#include "util/settings.h"
#include "control/interaction.h"

#include <cmath>
#include <limits>

#include <omp.h>

// required to get any printout when running from cmd.exe on Windows
#pragma comment(linker, "/SUBSYSTEM:CONSOLE")

using namespace std;

// Environment:

unsigned     Settings::WINDOW_WIDTH        = 720;
unsigned     Settings::WINDOW_HEIGHT       = 540;
bool         Settings::NO_PRINTOUT         = false;
bool         Settings::NO_SCREENS_NO_VIDEO = true; // eliminates inconsistent framerate, when enabled
std::string  Settings::IMG_FILE_TYPE       = ".png";
unsigned     Settings::VIDEO_FILE_FPS      = 120;

// Graphics:

bool          Settings::DOT_OR_SPHERE   = true;
unsigned      Settings::SPHERE_DETAIL   = 23;
bool          Settings::PAINT_VECTORS   = false;
unsigned char Settings::COLOR_BY        = ColorBy::BOUNDARIES;
double        Settings::VECTOR_LEN_MULT = 0.1; // 30.

// Controls:

unsigned char Settings::CONTROL_MODE = ONE_DRAG;

// Simulation:

unsigned char Settings::INTEGRATION_SCHEME   = REVERSE_EULER;
double        Settings::dt                   = 0.001; // 0.48 for forces limited to |5|
unsigned      Settings::ITERATIONS_PER_FRAME = 1;
double        Settings::ARENA_DIAMETER       = 33.; // 30. for 2000 0.15-radi particle tightly placed
double        Settings::ARENA_DIAMETER_Z     = 0.;
unsigned      Settings::X_PARTICLE_COUNT_3D  = 20;
unsigned      Settings::Y_PARTICLE_COUNT_3D  = 50;
unsigned      Settings::Z_PARTICLE_COUNT_3D  = 20;
unsigned      Settings::PARTICLE_COUNT_2D    = 2000; //X_PARTICLE_COUNT_3D * Y_PARTICLE_COUNT_3D; // 2000
unsigned char Settings::MAP_SETUP            = DAM_BREAK_NON_MAP;
unsigned      Settings::GHOST_LAYER_GAGE     = 0; // TODO
bool          Settings::CALCULATE_MASS       = false; // TODO
bool          Settings::PARALLEL_GPU         = false;
bool          Settings::PARALLEL_MPI         = false;
bool          Settings::PARALLEL_OMP         = true;
unsigned      Settings::PARALLEL_OMP_THREADS = omp_get_num_procs();
double        Settings::WORLD_ROTATION       = 0.;

// Physics:

bool   Settings::COLLIDE_NEIGHBOURS_ONLY = false; // as opposed to collide with all
bool   Settings::GRANULAR_OR_LIQUID      = true; // when granular, move one particle manually to eliminate equilibrial state
bool   Settings::PARTICLES_REACT         = true;
bool   Settings::COLLIDE_MOVE_OUT        = true;
bool   Settings::COLLIDE_TRANSFER_VEL    = true;
double Settings::PARTICLE_RADIUS         = 0.15; // 0.15
double Settings::PARTICLE_MAX_DR         = PARTICLE_RADIUS * 1000.5; // inf // 0.5;
double Settings::PARTICLES_INIT_DIST     = PARTICLE_RADIUS * 2.1; // 2.1;
double Settings::MESH_CELL_DIAMETER      = PARTICLE_RADIUS * 2.1;
                                         //= ARENA_DIAMETER;
double Settings::SMOOTHING_LENGTH        = PARTICLE_RADIUS * 16; // smoothing length / kernel support
//double Settings::PARTICLE_MASS           = 0.01;
double Settings::PARTICLE_MASS           = DESIRED_REST_DENSITY
                                           * pow(2./3 * SMOOTHING_LENGTH, 3);
//                                           * pow(Settings::SMOOTHING_LENGTH, 3);
//                                           * 10;

std::string   Settings::NEIGHBOUR_CHOICE      = "GRID"; // "ALL", "GRID", "OCTREE"
bool          Settings::NEIGHBOUR_LMT_BY_DST  = true;
double        Settings::NEIGHBOUR_RANGE       = PARTICLE_RADIUS * 128; // 2 * smoothing_length; // std::numeric_limits<double>::infinity();
int           Settings::NEIGHBOUR_RANGE_CELLS = 1;
unsigned char Settings::KERNEL_STEEPNESS      = QUINTIC_LIU;
unsigned      Settings::KERNEL_DIM            = 2; // TODO
double        Settings::STIFFNESS_CONSTANT    = 300.; // 300. // m/s
double        Settings::SOUND_SPEED           = 340.; // not in use
double        Settings::DESIRED_REST_DENSITY  = 1000.;
double        Settings::SURFACE_TENSION_COEF  = 1000.; // TODO
double        Settings::VISCOSITY             = 70; // 0.01 // 10^-6 but large values preffered for simulation
double        Settings::UNIVERSAL_GRAV_MULT   = 10000000000000000000.; // TODO

double Settings::FORCES_LIMIT             = 100000000; //////// TODO // 5.0, should be unlimited...
bool   Settings::FORCES_GRAVITY_EARTH     = false;
double Settings::FORCES_GRAVITY_EARTH_VAL = -9.81; // -9.81 // 0 // -0.13
bool   Settings::FORCES_GRAVITY_UNIVERSAL = false;
bool   Settings::FORCES_COULOMB           = false;
double Settings::FORCES_DAMPEN_WALL       = 0.05; // 1.0 - ELASTICITY
double Settings::FORCES_DAMPEN_WATER      = 0.00;
double Settings::FORCES_FRICTION          = 0.50;

// LEGACY //////////////////////////////////////////////////////////////////////

double Settings::FORCES_HOOKE_SPRING   = 0.7;
float  Settings::FORCES_HOOKE_DISTANCE = 0.3;
double Settings::FORCES_HOOKE_DAMP     = 0.;

float  Settings::WAVES_VELOCITY = 4.0;
float  Settings::WAVES_DT       = 0.05;
