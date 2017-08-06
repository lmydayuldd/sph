#ifndef INTERACTION_H
#define INTERACTION_H

class Particle;
class Vector;

class Interaction {
private:
    static Particle *pressedParticle;
    static Vector *lockedPressedParticlePosition;
    static Vector *pressedPoint;

public:
    static bool pause;
    static bool rewind;

    static void handleTouchPress(float normalizedX, float normalizedY);
    static void handleTouchDrag(float normalizedX, float normalizedY);
    static void holdPressedParticle();
    static void handleTouchDrop();
};

#endif // INTERACTION_H
