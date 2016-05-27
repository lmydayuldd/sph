#ifndef SIMULATION_WINDOW_H
#define SIMULATION_WINDOW_H

#include <QScreen>

class Timer;

#include "window/gl_window.h"

class SimulationWindow : public GLWindow
{
public:
    SimulationWindow();

    void initialize() Q_DECL_OVERRIDE;
    void render() Q_DECL_OVERRIDE;

    static Timer* timer;
    static int frame;
    static int refreshRate;
    static long long int formerTime;
    static long long int currentTime;
    static double dt;
    const float moveSpeed = 0.000000005f;//10.f;
    const float mouseSpeed = 0.3f;
    float cursorX, cursorY;

private:
    std::string previousAction;
    float previousX;
    float previousY;
    float previousDistance;
    float t = 1.0f;

    void keyPressEvent(QKeyEvent* e);
    void keyReleaseEvent(QKeyEvent* e);
    void mouseMoveEvent(QMouseEvent* e);

    static void cameraMan();
    void move();
    void w();
    void s();
    void a();
    void d();
    void h();
    void l();
    void r();
};

#endif // SIMULATION_WINDOW_H
