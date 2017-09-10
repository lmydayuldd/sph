#ifndef SETTINGS_H
#define SETTINGS_H

#include <string>

class Settings {
public:
    static const int BYTES_PER_FLOAT = 4; // used by VertexArray

    // Environment:

    static unsigned    WINDOW_WIDTH;
    static unsigned    WINDOW_HEIGHT;
    static bool        NO_PRINTOUT;
    static bool        NO_SCREENS_NO_VIDEO;
    static std::string IMG_FILE_TYPE;
    static unsigned    VIDEO_FILE_FPS;

    // Graphics:

    static bool          DOT_OR_SPHERE;
    static unsigned      SPHERE_DETAIL;
    static bool          PAINT_VECTORS;
    static unsigned char COLOR_BY;
    static double        VECTOR_LEN_MULT;

    // Controls:

    static unsigned char CONTROL_MODE;

    // Simulation:

    static unsigned char INTEGRATION_SCHEME;
    static double        dt;
    static unsigned      ITERATIONS_PER_FRAME;
    static double        ARENA_DIAMETER;
    static double        ARENA_DIAMETER_Z;
    static unsigned      X_PARTICLE_COUNT_3D;
    static unsigned      Y_PARTICLE_COUNT_3D;
    static unsigned      Z_PARTICLE_COUNT_3D;
    static unsigned      PARTICLE_COUNT_2D;
    static unsigned char MAP_SETUP;
    static unsigned      GHOST_LAYER_GAGE;
    static bool          CALCULATE_MASS;
    static bool          PARALLEL_GPU;
    static bool          PARALLEL_MPI;
    static bool          PARALLEL_OMP;
    static unsigned      PARALLEL_OMP_THREADS;
    static double        WORLD_ROTATION;

    // Physics:

    static bool          COLLIDE_NEIGHBOURS_ONLY;
    static bool          GRANULAR_OR_LIQUID;
    static bool          PARTICLES_REACT;
    static bool          COLLIDE_MOVE_OUT;
    static bool          COLLIDE_TRANSFER_VEL;
    static double        PARTICLE_RADIUS;
    static double        PARTICLE_MAX_DR;
    static double        PARTICLES_INIT_DIST;
    static double        MESH_CELL_DIAMETER;
    static double        SMOOTHING_LENGTH;
    static double        PARTICLE_MASS;
    static std::string   NEIGHBOUR_CHOICE;
    static bool          NEIGHBOUR_LMT_BY_DST;
    static double        NEIGHBOUR_RANGE;
    static int           NEIGHBOUR_RANGE_CELLS;
    static unsigned char KERNEL_STEEPNESS;
    static unsigned      KERNEL_DIM;
    static double        STIFFNESS_CONSTANT;
    static double        SOUND_SPEED;
    static double        DESIRED_REST_DENSITY;
    static double        SURFACE_TENSION_COEF;
    static double        VISCOSITY;
    static double        UNIVERSAL_GRAV_MULT;

    static double FORCES_LIMIT;
    static bool   FORCES_GRAVITY_EARTH;
    static double FORCES_GRAVITY_EARTH_VAL;
    static bool   FORCES_GRAVITY_UNIVERSAL;
    static bool   FORCES_COULOMB;
    static double FORCES_FRICTION;
    static double FORCES_DAMPEN_WALL; // 1.0 - ELASTICITY
    static double FORCES_DAMPEN_WATER;

// LEGACY //////////////////////////////////////////////////////////////////////

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
