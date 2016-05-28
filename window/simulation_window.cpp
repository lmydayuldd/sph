#include "window/simulation_window.h"

#include <QScreen>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QMessageBox>

#include <iostream>

class Particle;

#include "gl/form.h"
#include "gl/handles.h"
#include "gl/matrices.h"
#include "gl/vertex_array.h"
#include "machine/walls.h"
#include "physics/computer.h"
#include "shader/shader.h"
#include "util/constants.h"
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

    refreshRate = screen()->refreshRate();
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);

    //    Matrices::viewMatrix.translate(QVector3D(10, 10, 10));
    Matrices::camTZ = 12;
    Matrices::camRY = 180;
    Matrices::projectionMatrix.setToIdentity();
    Matrices::projectionMatrix.perspective(90.0f, width()/height(), 0.1f, 100.0f);

    Particle::flows.push_back(std::vector<Particle*>(Settings::PARTICLE_COUNT));
    for (unsigned int i = 0; i < Particle::flows[0].size(); ++i) {
        Particle::flows[0][i] = new Particle(&Particle::flows[0]);
    }
    Machine::machines.push_back(new Walls(10.0));//getDamping("static")));

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
    //std::cout << "~" << floor(1000000000 / dt) << " FPS" << std::endl;

    Computer::currentComputer->loop();
    move();

    Shader::currentShader->program->bind();
    {
        Matrices::viewMatrix.setToIdentity();
        Matrices::viewProjectionMatrix.setToIdentity();
        Matrices::viewProjectionInverted.setToIdentity();
        Matrices::setViewMatrix();
        Matrices::viewProjectionMatrix   = Matrices::projectionMatrix * Matrices::viewMatrix;
        Matrices::viewProjectionInverted = Matrices::viewProjectionMatrix.inverted();

        Form::printForms();
        VertexArray::printArrays();
        for (unsigned int i = 0; i < Particle::flows[0].size(); ++i) {
            Particle::flows[0][i]->paint();
        }
        for (unsigned int i = 0; i < Machine::machines.size(); ++i) {
            Machine::machines[i]->paint();
        }
    }
    Shader::currentShader->program->release();

    ++frame;
}

bool key[6] = {0};
void SimulationWindow::move() {
    if (key[0]) w();
    if (key[1]) s();
    if (key[2]) a();
    if (key[3]) d();
    if (key[4]) h();
    if (key[5]) l();
}

void SimulationWindow::keyPressEvent(QKeyEvent* e)
{
//    QMessageBox* box = new QMessageBox();
//    box->setWindowTitle(QString("Hello"));
//    box->setText(QString("You Pressed: ") + event->text());
//    box->show();

    switch (e->key())
    {
        case Qt::Key_W     : key[0] = true; break;
        case Qt::Key_Up    : key[0] = true; break;
        case Qt::Key_S     : key[1] = true; break;
        case Qt::Key_Down  : key[1] = true; break;
        case Qt::Key_A     : key[2] = true; break;
        case Qt::Key_Left  : key[2] = true; break;
        case Qt::Key_D     : key[3] = true; break;
        case Qt::Key_Right : key[3] = true; break;
        case Qt::Key_Space : key[4] = true; break;
        case Qt::Key_V     : key[5] = true; break;
    }
}

void SimulationWindow::keyReleaseEvent(QKeyEvent *e)
{
    if (! e->isAutoRepeat()) {
        switch (e->key()) {
            case Qt::Key_W     : key[0] = false; break;
            case Qt::Key_S     : key[1] = false; break;
            case Qt::Key_A     : key[2] = false; break;
            case Qt::Key_D     : key[3] = false; break;
            case Qt::Key_Space : key[4] = false; break;
            case Qt::Key_V     : key[5] = false; break;
        }
    }
}

void SimulationWindow::mouseMoveEvent(QMouseEvent* event) {
    QPointF p = event->pos();
    int dy = (width()  / 2) - p.x();
    int dx = (height() / 2) - p.y();

    Matrices::camRX -= dx * mouseSpeed;
    Matrices::camRY -= dy * mouseSpeed;
    if (Matrices::camRX >  90) Matrices::camRX =  90;
    if (Matrices::camRX < -90) Matrices::camRX = -90;

    QCursor::setPos(
        geometry().x() + width()/2,
        geometry().y() + height()/2
    );
}

void SimulationWindow::w() {
    Matrices::camTX -= moveSpeed * sin(Matrices::camRY * Constants::degToRad) * cos(Matrices::camRX * Constants::degToRad) * dt;
    Matrices::camTY += moveSpeed * sin(Matrices::camRX * Constants::degToRad) * dt;
    Matrices::camTZ += moveSpeed * cos(Matrices::camRY * Constants::degToRad) * cos(Matrices::camRX * Constants::degToRad) * dt;
}
void SimulationWindow::s() {
    Matrices::camTX += moveSpeed * sin(Matrices::camRY * Constants::degToRad) * cos(Matrices::camRX * Constants::degToRad) * dt;
    Matrices::camTY -= moveSpeed * sin(Matrices::camRX * Constants::degToRad) * dt;
    Matrices::camTZ -= moveSpeed * cos(Matrices::camRY * Constants::degToRad) * cos(Matrices::camRX * Constants::degToRad) * dt;
}
void SimulationWindow::a() {
    Matrices::camTX += moveSpeed * cos(Matrices::camRY * Constants::degToRad) * dt;
    Matrices::camTZ += moveSpeed * sin(Matrices::camRY * Constants::degToRad) * dt;
}
void SimulationWindow::d() {
    Matrices::camTX -= moveSpeed * cos(Matrices::camRY * Constants::degToRad) * dt;
    Matrices::camTZ -= moveSpeed * sin(Matrices::camRY * Constants::degToRad) * dt;
}
void SimulationWindow::h() {
    Matrices::cam[1] += t;
    Matrices::cam[4] += t;
}
void SimulationWindow::l() {
    Matrices::cam[1] -= t;
    Matrices::cam[4] -= t;
}
void SimulationWindow::r() {
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
