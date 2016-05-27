#ifndef INTERACTION_H
#define INTERACTION_H

class Particle;

class Interaction {
private:
    static Particle* pressedParticle;

public:
    static void handleTouchPress(float normalizedX, float normalizedY);
    static void handleTouchDrag(float normalizedX, float normalizedY);
    static void handleTouchDrop();
};

#endif // INTERACTION_H
