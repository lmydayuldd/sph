#ifndef SIMULATION_WINDOW_H
#define SIMULATION_WINDOW_H

#include <QScreen>

class Timer;
class QImage;

#include "window/gl_window.h"

class SimulationWindow : public GLWindow
{
public:
    static SimulationWindow *simWin;
    static Timer *frameTimer;
    static int frame;
    static int refreshRate;
    static long long int frameStartTime;
    static long long int currentTime;
    static double frame_dt;

    SimulationWindow();
    ~SimulationWindow();

    static void saveVideo();

    void prepareSimulation();
    void initialize() Q_DECL_OVERRIDE;
    void render() Q_DECL_OVERRIDE;

private:
    static std::vector<QImage> screens;

    void keyPressEvent(QKeyEvent *e);
    void keyReleaseEvent(QKeyEvent *e);
    void mouseMoveEvent(QMouseEvent *e);
    void mousePressEvent(QMouseEvent *e);
    void mouseReleaseEvent(QMouseEvent *e);
};

#endif // SIMULATION_WINDOW_H
