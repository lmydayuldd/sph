#ifndef SETTINGS_H
#define SETTINGS_H

class Settings {
public:
    static const int BYTES_PER_FLOAT = 4; // used by VertexArray

    static unsigned int PARTICLE_COUNT;    // used by Particle
    static unsigned int SPHERE_DETAIL;    // used by Particle, CollisionSphere
    static bool         PAINT_VECTORS;  // used by Vector
    static bool         PARTICLES_REACT; // used by Particle, ?glRenderer?
    static bool         DIMENSIONS_3D;

    static float  dt;
    static float  DAMPENING; // 1.0 - ELASTICITY

    static bool   FORCES_EARTH_GRAVITY;
    static bool   FORCES_UNIVERSAL_GRAVITY;
    static bool   FORCES_COULOMB;

    static double FORCES_HOOKE_SPRING;
    static float  FORCES_HOOKE_DISTANCE;
    static double FORCES_HOOKE_DAMP;

    static float  FORCES_FRICTION;

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
