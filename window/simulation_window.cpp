#include "window/simulation_window.h"

#include <QScreen>
#include <QKeyEvent>
#include <QMouseEvent>

#include <iostream>

class Particle;

#include "control/interaction.h"
#include "gl/form.h"
#include "gl/handles.h"
#include "gl/matrices.h"
#include "gl/vertex_array.h"
#include "machine/cloth.h"
#include "machine/rope.h"
#include "machine/walls.h"
#include "physics/computer.h"
#include "physics/grid.h"
#include "shader/shader.h"
#include "util/constants.h"
#include "util/debug_helper.h"
#include "util/timer.h"
#include "util/settings.h"

Timer* SimulationWindow::timer = new Timer();
long long int SimulationWindow::formerTime = 0;
long long int SimulationWindow::currentTime = 0;
double SimulationWindow::dt = 0;
int SimulationWindow::frame = 0;
int SimulationWindow::refreshRate = 0;

SimulationWindow::SimulationWindow()
{
}

void SimulationWindow::initialize()
{
    Shader::currentShader = new Shader();
    Computer::currentComputer = new Computer();
    Grid::init();

    refreshRate = screen()->refreshRate();
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);

    //    Matrices::viewMatrix.translate(QVector3D(10, 10, 10));
    Matrices::camTZ = 12;
    Matrices::camRY = 0;
    Matrices::projectionMatrix.setToIdentity();
    Matrices::projectionMatrix.perspective(
                                    90.0f, width()/height(), 0.1f, 100.0f);

    Particle::flows.push_back(std::vector<Particle*>(Settings::PARTICLE_COUNT));
    for (unsigned int i = 0; i < Particle::flows[0].size(); ++i)
    {
        Particle::flows[0][i] = new Particle(Particle::flows.size() - 1);
    }
    Machine::machines.push_back(
            new Walls(Settings::ARENA_DIAMETER / 2)
    );//getDamping("static")));
//    Machine::machines.push_back(
//        new Rope(
//            Vector(-6, 3, 0), Vector(6, 3, 0),
//            30, 5,
//            300, Settings::PARTICLE_RADIUS * 2, 120
//        )
//    );
//    Machine::machines.push_back(
//        new Cloth(
//            Vector(-2, -2, 0), Vector(2, 2, 0),
//            6, 300, 0.001, 12, -1
//        )
//    );

    QCursor::setPos(geometry().x() + width()/2, geometry().y() + height()/2);

    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);
}

void SimulationWindow::render()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);

    currentTime = timer->diff();
    dt          = currentTime - formerTime;
    formerTime  = currentTime;
    std::cout << "~" << floor(1000000000 / dt) << " FPS" << std::endl;

    Computer::currentComputer->loop();
    move();
    Interaction::holdPressedParticle();

    Shader::currentShader->program->bind();
    {
        Matrices::viewMatrix.setToIdentity();
        Matrices::viewProjectionMatrix.setToIdentity();
        Matrices::viewProjectionInverted.setToIdentity();
        Matrices::setViewMatrix();
        Matrices::viewProjectionMatrix
                = Matrices::projectionMatrix * Matrices::viewMatrix;
        Matrices::viewProjectionInverted
                = Matrices::viewProjectionMatrix.inverted();

//        Form::printForms();
//        VertexArray::printArrays();
        for (unsigned int i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i]->paint();
        }
        for (unsigned int i = 0; i < Machine::machines.size(); ++i)
        {
            Machine::machines[i]->paint();
        }
    }
    Shader::currentShader->program->release();

    ++frame;
}

enum Key : unsigned char {
    W, S, A, D,
    JUMP, DUCK,
    CTRL, LMB, RMB
};

bool key[sizeof(unsigned char) * 256] = {0};
void SimulationWindow::move()
{
    if (key[W])    w();
    if (key[S])    s();
    if (key[A])    a();
    if (key[D])    d();
    if (key[JUMP]) h();
    if (key[DUCK]) l();
}

void SimulationWindow::keyPressEvent(QKeyEvent* e)
{
    switch (e->key())
    {
        case Qt::Key_W       : key[W]    = true; break;
        case Qt::Key_Up      : key[W]    = true; break;
        case Qt::Key_S       : key[S]    = true; break;
        case Qt::Key_Down    : key[S]    = true; break;
        case Qt::Key_A       : key[A]    = true; break;
        case Qt::Key_Left    : key[A]    = true; break;
        case Qt::Key_D       : key[D]    = true; break;
        case Qt::Key_Right   : key[D]    = true; break;
        case Qt::Key_Space   : key[JUMP] = true; break;
        case Qt::Key_V       : key[DUCK] = true; break;
        case Qt::Key_Control : key[CTRL] = true; break;
    }
}

void SimulationWindow::keyReleaseEvent(QKeyEvent *e)
{
    if (! e->isAutoRepeat())
    {
        switch (e->key())
        {
            case Qt::Key_W       : key[W]    = false; break;
            case Qt::Key_S       : key[S]    = false; break;
            case Qt::Key_A       : key[A]    = false; break;
            case Qt::Key_D       : key[D]    = false; break;
            case Qt::Key_Space   : key[JUMP] = false; break;
            case Qt::Key_V       : key[DUCK] = false; break;
            case Qt::Key_Control :
                key[CTRL] = false;
                Interaction::handleTouchDrop();
            break;
        }
    }
}

void SimulationWindow::mouseMoveEvent(QMouseEvent* event)
{
    mousePoint = event->pos();
    normalizedX =    (mousePoint.x() / (float) width())  * 2 - 1;
    normalizedY = - ((mousePoint.y() / (float) height()) * 2 - 1);
    dy = (width()  / 2) - mousePoint.x();
    dx = (height() / 2) - mousePoint.y();

    if (! key[CTRL])
    {
        Matrices::camRX -= dx * mouseSpeed;
        Matrices::camRY -= dy * mouseSpeed;
        if (Matrices::camRX >  90) Matrices::camRX =  90;
        if (Matrices::camRX < -90) Matrices::camRX = -90;

        QCursor::setPos(geometry().x() + width()/2,
                        geometry().y() + height()/2);
    }
    else
    {
        if (key[LMB])
        {
            Interaction::handleTouchDrag(normalizedX, normalizedY);
        }
    }
}

void SimulationWindow::mousePressEvent(QMouseEvent* event)
{
    mousePoint = event->pos();
    normalizedX =    (mousePoint.x() / (float) width())  * 2 - 1;
    normalizedY = - ((mousePoint.y() / (float) height()) * 2 - 1);

    if (key[CTRL])
    {
        switch(event->button())
        {
            case Qt::LeftButton :
                key[LMB] = true;
                Interaction::handleTouchPress(normalizedX, normalizedY);
            break;
            case Qt::RightButton :
                key[RMB] = true;
            break;
            default : break;
        }
    }
}
void SimulationWindow::mouseReleaseEvent(QMouseEvent* event)
{
    if (key[CTRL])
    {
        switch(event->button())
        {
            case Qt::LeftButton :
                key[LMB] = false;
//                DebugHelper::showVariable("normx", normalizedX);
//                DebugHelper::showVariable("normy", normalizedY);
//                DebugHelper::showVariables("aaaa", 12, 13, 15);
                Interaction::handleTouchDrop();
            break;
            case Qt::RightButton :
                key[RMB] = false;
            break;
            default : break;
        }
    }
}

void SimulationWindow::w()
{
    Matrices::camTX += moveSpeed * sin(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * dt;
    Matrices::camTY -= moveSpeed * sin(Matrices::camRX * degToRad) * dt;
    Matrices::camTZ -= moveSpeed * cos(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * dt;
}
void SimulationWindow::s()
{
    Matrices::camTX -= moveSpeed * sin(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * dt;
    Matrices::camTY += moveSpeed * sin(Matrices::camRX * degToRad) * dt;
    Matrices::camTZ += moveSpeed * cos(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * dt;
}
void SimulationWindow::a()
{
    Matrices::camTX -= moveSpeed * cos(Matrices::camRY * degToRad) * dt;
    Matrices::camTZ -= moveSpeed * sin(Matrices::camRY * degToRad) * dt;
}
void SimulationWindow::d()
{
    Matrices::camTX += moveSpeed * cos(Matrices::camRY * degToRad) * dt;
    Matrices::camTZ += moveSpeed * sin(Matrices::camRY * degToRad) * dt;
}
void SimulationWindow::h()
{
    Matrices::cam[1] += t;
    Matrices::cam[4] += t;
}
void SimulationWindow::l()
{
    Matrices::cam[1] -= t;
    Matrices::cam[4] -= t;
}
void SimulationWindow::r()
{
    Matrices::viewMatrix.setToIdentity();
    Matrices::viewMatrix.lookAt(
        QVector3D(Matrices::defaultCam[0], Matrices::defaultCam[1], Matrices::defaultCam[2]),
        QVector3D(Matrices::defaultCam[3], Matrices::defaultCam[4], Matrices::defaultCam[5]),
        QVector3D(Matrices::defaultCam[6], Matrices::defaultCam[7], Matrices::defaultCam[8])
    );
    Matrices::camTX = 0;
    Matrices::camTY = 0;
    Matrices::camTZ = 0;
    Matrices::camRX = 0;
    Matrices::camRY = 0;
    Matrices::setViewMatrix();
}
