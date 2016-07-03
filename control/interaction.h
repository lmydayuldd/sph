#ifndef INTERACTION_H
#define INTERACTION_H

class Particle;
class Vector;

class Interaction {
private:
    static Particle* pressedParticle;
    static Vector* lockedPressedParticlePosition;

public:
    static void handleTouchPress(float normalizedX, float normalizedY);
    static void handleTouchDrag(float normalizedX, float normalizedY);
    static void holdPressedParticle();
    static void handleTouchDrop();
};

#endif // INTERACTION_H
