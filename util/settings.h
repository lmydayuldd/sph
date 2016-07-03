#ifndef SETTINGS_H
#define SETTINGS_H

class Settings {
public:
    static const int BYTES_PER_FLOAT = 4; // used by VertexArray

    static unsigned int SPHERE_DETAIL;
    static bool         PAINT_VECTORS;
    static float        VECTOR_LENGTH_MULTIPLIER;

    static unsigned int PARTICLE_COUNT;
    static bool         PARTICLES_REACT;
    static bool         DIMENSIONS_3D;
    static bool         PARTICLES_INITIAL_SPEED;

    static float PARTICLE_MASS;
    static float PARTICLE_RADIUS;

    static bool   FORCES_EARTH_GRAVITY;
    static bool   FORCES_UNIVERSAL_GRAVITY;
    static bool   FORCES_COULOMB;
    static double FORCES_HOOKE_SPRING;
    static float  FORCES_HOOKE_DISTANCE;
    static double FORCES_HOOKE_DAMP;
    static float  FORCES_FRICTION;
    static float  DAMPENING; // 1.0 - ELASTICITY

    static float  dt;
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
