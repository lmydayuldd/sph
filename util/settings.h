#ifndef SETTINGS_H
#define SETTINGS_H

class Settings {
public:
    static const int BYTES_PER_FLOAT = 4; // used by VertexArray

    // Graphics:

    static bool         DOT_OR_SPHERE;
    static unsigned int SPHERE_DETAIL;
    static bool         PAINT_VECTORS;
    static bool         COLOR_BY_SPEED;
    static bool         COLOR_BOUNDARIES;
    static double       VECTOR_LENGTH_MULTIPLIER;

    // Simulation:

    static double       dt;
    static float        WAVES_VELOCITY;
    static float        WAVES_DT;
    static double       ARENA_DIAMETER;
    static unsigned int PARTICLE_COUNT;
    static bool         PARTICLES_REACT;

    // Physics:

    static double PARTICLE_MASS;
    static double PARTICLE_RADIUS;
    static double GRAVITY_VALUE;

    static bool         SPH_NEIGHBOUR_BY_CNT_OR_DIST;
    static unsigned int SPH_NEIGHBOUR_COUNT;
    static unsigned int SPH_KERNEL_DIM;
    static double       SPH_PARTICLES_INIT_DIST;
    static double       SPH_PARTICLE_MAX_DR;
    static double       SPH_MESH_CELL_DIAMETER;
    static double       SPH_SMOOTHING_LENGTH;
    static double       SPH_NEIGHBOUR_RANGE;
    static double       SPH_STIFFNESS_CONSTANT;
    static double       SPH_DESIRED_REST_DENSITY;
    static double       SPH_VISCOSITY;
    static double       SPH_PRESSURE;
    static double       SPH_TEMPERATURE;

    static bool   FORCES_EARTH_GRAVITY;
    static bool   FORCES_UNIVERSAL_GRAVITY;
    static bool   FORCES_COULOMB;
    static double FORCES_HOOKE_SPRING;
    static float  FORCES_HOOKE_DISTANCE;
    static double FORCES_HOOKE_DAMP;
    static double FORCES_FRICTION;
    static double WALL_DAMPENING; // 1.0 - ELASTICITY
    static double WATER_DAMPENING;

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

    // Controls:

    static bool DRAG_OR_FORCE;
};

#endif // SETTINGS_H
