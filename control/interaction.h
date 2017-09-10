#ifndef INTERACTION_H
#define INTERACTION_H

#include <QKeyEvent>

class Particle;
class Vector;

class Interaction {
private:
    static Particle *pressedParticle;
    static Vector *lockedPressedParticlePosition;
    static Vector *pressedPoint;
    static QPoint *mousePoint;

    static float t;
    static int dx, dy;
    static float normalizedX, normalizedY;
    static float cursorX, cursorY;
    static float previousX, previousY;
    static float previousDistance;
    static std::string previousAction;

public:
    static bool pause;
    static bool rewind;
    static float moveSpeed;
    static float mouseSpeed;
    static bool key[sizeof(unsigned char) * 256];

    static void handleTouchPress(float normalizedX, float normalizedY);
    static void handleTouchDrag(float normalizedX, float normalizedY);
    static void holdPressedParticle();
    static void handleTouchDrop();

    static void keyPress(QKeyEvent *e);
    static void keyRelease(QKeyEvent *e);
    static void mouseMove(QMouseEvent *e);
    static void mousePress(QMouseEvent *e);
    static void mouseRelease(QMouseEvent *e);

    static void interact();
    static void w();
    static void s();
    static void a();
    static void d();
    static void h();
    static void l();
    static void r();
};

#endif // INTERACTION_H
