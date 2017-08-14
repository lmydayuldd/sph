#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>

class Settings {
public:
    static const int BYTES_PER_FLOAT = 4; // used by VertexArray

    // Environment:

    static unsigned WINDOW_WIDTH;
    static unsigned WINDOW_HEIGHT;

    // Graphics:

    static bool     DOT_OR_SPHERE;
    static unsigned SPHERE_DETAIL;
    static bool     PAINT_VECTORS;
    static bool     COLOR_BY_SPEED;
    static bool     COLOR_BOUNDARIES;
    static double   VECTOR_LENGTH_MULTIPLIER;

    // Controls:

    static unsigned char CONTROL_MODE;

    // Simulation:

    static double        dt;
    static unsigned      ITERATIONS_PER_FRAME;
    static double        ARENA_DIAMETER;
    static double        ARENA_DIAMETER_Z;
    static unsigned      PARTICLE_COUNT;
    static unsigned char MAP_SETUP;
    static unsigned      GHOST_LAYER_GAGE;
    static bool          PARALLEL_GPU;
    static bool          PARALLEL_MPI;
    static bool          PARALLEL_OMP;
    static unsigned      PARALLEL_OMP_THREADS;
    static unsigned      PARTICLES_X;
    static unsigned      PARTICLES_Y;
    static unsigned      PARTICLES_Z;

    // Physics:

    static bool         PARTICLES_REACT;
    static bool         COLLIDE_MOVE_OUT;
    static bool         COLLIDE_TRANSFER_VEL;
    static bool         NEIGHBOUR_LMT_BY_CNT;
    static bool         NEIGHBOUR_LMT_BY_DST;
    static std::string  NEIGHBOUR_CHOICE;
    static unsigned     NEIGHBOUR_COUNT;
    static unsigned     KERNEL_DIM;
    static double       STIFFNESS_CONSTANT;
    static double       DESIRED_REST_DENSITY;
    static double       VISCOSITY;
    static double       PARTICLE_RADIUS;
    static double       PARTICLES_INIT_DIST;
    static double       MESH_CELL_DIAMETER;
    static double       SMOOTHING_LENGTH;
    static double       NEIGHBOUR_RANGE;
    static double       PARTICLE_MAX_DR;
    static double       PARTICLE_MASS;

    static bool   FORCES_GRAVITY_EARTH;
    static double FORCES_GRAVITY_EARTH_VAL;
    static bool   FORCES_GRAVITY_UNIVERSAL;
    static bool   FORCES_COULOMB;
    static double FORCES_FRICTION;
    static double FORCES_DAMPEN_WALL; // 1.0 - ELASTICITY
    static double FORCES_DAMPEN_WATER;

// LEGACY //////////////////////////////////////////////////////////////////////

    static double PRESSURE;
    static double TEMPERATURE;

    static double FORCES_HOOKE_SPRING;
    static float  FORCES_HOOKE_DISTANCE;
    static double FORCES_HOOKE_DAMP;

    static float  WAVES_VELOCITY;
    static float  WAVES_DT;

//    static void setWavesVelocity(int v);
//    static void setWavesDT(float dt);
//    static void setWavesCells(int count);
//    static void setWavesWidth(int width);

//    static float getRadius(); // taken at Flow
//    static double getMass(); // taken at Flow
//    static double getCharge();
//    static float getHeat();
//    static float getDamping(String object); // taken at StaticObstacle, Particle

//    static double[] getStartingPosition(); // taken at Flow
//    static double[] getStartingVelocity(); // taken at Flow
};

#endif // SETTINGS_H
