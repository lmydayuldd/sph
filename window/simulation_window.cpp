#include "window/simulation_window.h"

#include <QScreen>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QString>
#include <QPainter>
#include <QApplication>

#include <iostream>

#include <omp.h>
#include "mpi.h"

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
#include "util/map.h"
#include "util/timer.h"
#include "util/settings.h"

Timer* SimulationWindow::timer = new Timer();
long long int SimulationWindow::formerTime = 0;
long long int SimulationWindow::currentTime = 0;
double SimulationWindow::dt = 0;
int SimulationWindow::frame = 0;
int SimulationWindow::refreshRate = 0;
bool SimulationWindow::key[] = {0};

SimulationWindow::SimulationWindow()
{
//     MPI_Init(NULL, NULL);
//     MPI_Finalize();
}

SimulationWindow::~SimulationWindow()
{
    std::cout << "Exiting the program." << std::flush;
}

void SimulationWindow::prepareSimulation()
{
    omp_set_num_threads(Settings::PARALLEL_OMP_THREADS);

    Grid::init();

    Map::generate();

    for (unsigned i = 0; i < Settings::PARTICLE_COUNT; ++i)
    {
        Particle::collision.push_back(std::vector<bool>(Settings::PARTICLE_COUNT));
        Particle::collisionDistance.push_back(std::vector<double>(Settings::PARTICLE_COUNT));
        for (unsigned j = 0; j < Settings::PARTICLE_COUNT; ++j)
        {
            Particle::collision[i][j] = false;
            Particle::collisionDistance[i][j] = std::numeric_limits<double>::infinity();
        }
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

    if (Settings::GHOST_LAYER_GAGE > 0)
    {
//#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i]->r->x += Settings::GHOST_LAYER_GAGE
                                         * Settings::PARTICLES_INIT_DIST;
            Particle::flows[0][i]->r->y += Settings::GHOST_LAYER_GAGE
                                         * Settings::PARTICLES_INIT_DIST;
        }
        Particle *ghostParticle;
        for (unsigned i = 0; i < Grid::cell_count; ++i)
        {
//#pragma omp parallel for if(Settings::PARALLEL_OMP)
            for (unsigned j = 0; j < Grid::cell_count; ++j)
            {
                if (i < Settings::GHOST_LAYER_GAGE
                 || j < Settings::GHOST_LAYER_GAGE
                 || i >= Grid::cell_count - Settings::GHOST_LAYER_GAGE
                 || j >= Grid::cell_count - Settings::GHOST_LAYER_GAGE)
                {
                    ghostParticle = new Particle(0);
                    ghostParticle->stationary = true;
                    ghostParticle->r->x = - Settings::ARENA_DIAMETER/2
                                          + Settings::PARTICLE_RADIUS
                                          + i * Settings::PARTICLES_INIT_DIST;
                    ghostParticle->r->y =   Settings::ARENA_DIAMETER/2
                                          - Settings::PARTICLE_RADIUS
                                          - j * Settings::PARTICLES_INIT_DIST;
    //#pragma omp atomic // critical
                    Particle::flows[0].push_back(ghostParticle);
                }
            }
        }
    }
}

void SimulationWindow::initialize()
{
    Shader::currentShader = new Shader();
    Computer::currentComputer = new Computer();

    refreshRate = screen()->refreshRate(); // frame // fps // frequency // period
    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);

    //    Matrices::viewMatrix.translate(QVector3D(10, 10, 10));
    Matrices::camTZ = 12;
    Matrices::camRY = 0;
    Matrices::projectionMatrix.setToIdentity();
    Matrices::projectionMatrix.perspective(
                                    90.0f, width()/height(), 0.1f, 100.0f);

    prepareSimulation();

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
    if (Interaction::rewind)
    {
        dt *= -1;
    }
    if (! Interaction::pause || SimulationWindow::key[Interaction::RENDER]) {
        std::cout << round(dt/1000000.0) << "ms <- Frame time." << std::endl;
        std::cout << "~" << round(1000000000 / dt) << " FPS" << std::endl;
        std::cout << "-- Frame " << frame-1 << " end ---------------------------" << std::endl << std::endl;
    }

    if (! Interaction::pause || SimulationWindow::key[Interaction::RENDER])
        std::cout << "-- Frame " << frame << " start --------------------------" << std::endl;
#pragma omp parallel if(Settings::PARALLEL_OMP)
    {
#pragma omp master
    if (! Interaction::pause || SimulationWindow::key[Interaction::RENDER])
        std::cout << omp_get_num_threads() << " threads." << std::endl;
    }
    interact(); //////////////////////////////////////////////
    if (! Interaction::pause || SimulationWindow::key[Interaction::RENDER])
    {
        Computer::currentComputer->loop();
    }

#pragma omp master
{
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

//#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
        {
            Particle::flows[0][i]->paint();
        }
//#pragma omp parallel for if(Settings::PARALLEL_OMP)
        for (unsigned i = 0; i < Machine::machines.size(); ++i)
        {
            Machine::machines[i]->paint();
        }
    }
    if (frame % 30 == 0 && frame/30 < 50)
    {
//        char pixels[width() * height() * 4];
//        glReadPixels(0, 0, width(), height(), GL_RGBA, GL_UNSIGNED_BYTE, pixels);
//        QFile *file = new QFile(QString("raw_screenshot_%1.bmp").arg(frame/30));
//        file->open(QIODevice::WriteOnly);
//        file->write(pixels, width() * height() * 4);
//        file->close();
//        delete file;

//        QImage img(size(), QImage::Format_ARGB32);
//        //img.fill(Qt::white);
//        QPainter painter(&img);
//        painter.setRenderHint(QPainter::Antialiasing);
//        render(&painter);
//        img.save(QString("raw_screenshot_%1.bmp").arg(frame/30));

        QPixmap pixMap = QPixmap::grabWindow(winId(), x() - width()/2, y() - height()/2, width(), height());
        QImage img = pixMap.toImage();
        QPainter painter(&img);
        img.save(QString("raw_screenshot_%1.bmp").arg(frame/30));
    }
    Shader::currentShader->program->release();
}

#pragma omp master
    if (! Interaction::pause || SimulationWindow::key[Interaction::RENDER])
        ++frame;
}

void SimulationWindow::interact()
{
    if (key[Interaction::W])    w();
    if (key[Interaction::S])    s();
    if (key[Interaction::A])    a();
    if (key[Interaction::D])    d();
    if (key[Interaction::DUCK]) l();
    Interaction::holdPressedParticle();
}

void SimulationWindow::keyPressEvent(QKeyEvent* e)
{
    switch (e->key())
    {
        case Qt::Key_W         : key[Interaction::W]         = true; break;
        case Qt::Key_Up        : key[Interaction::W]         = true; break;
        case Qt::Key_S         : key[Interaction::S]         = true; break;
        case Qt::Key_Down      : key[Interaction::S]         = true; break;
        case Qt::Key_A         : key[Interaction::A]         = true; break;
        case Qt::Key_Left      : key[Interaction::A]         = true; break;
        case Qt::Key_D         : key[Interaction::D]         = true; break;
        case Qt::Key_Right     : key[Interaction::D]         = true; break;
        case Qt::Key_V         : key[Interaction::DUCK]      = true; break;
        case Qt::Key_Control   : key[Interaction::CTRL]      = true; break;
        case Qt::Key_Space     : key[Interaction::SPACE]     = true; break;
        case Qt::Key_Backspace : key[Interaction::BACKSPACE] = true; break;
        case Qt::Key_Escape    : key[Interaction::ESCAPE]    = true; break;
        case Qt::Key_P         : key[Interaction::PARALLEL]  = true; break;
//        case Qt::Key_R         : key[Interaction::RENDER]    = true; break;
        case Qt::Key_C         : key[Interaction::CONTROL]   = true; break;
    }
}

void SimulationWindow::keyReleaseEvent(QKeyEvent *e)
{
    if (! e->isAutoRepeat())
    {
        switch (e->key())
        {
            case Qt::Key_W         : key[Interaction::W]     = false; break;
            case Qt::Key_S         : key[Interaction::S]     = false; break;
            case Qt::Key_A         : key[Interaction::A]     = false; break;
            case Qt::Key_D         : key[Interaction::D]     = false; break;
            case Qt::Key_V         : key[Interaction::DUCK]  = false; break;
            case Qt::Key_Control   :
                key[Interaction::CTRL] = false;
                Interaction::handleTouchDrop();
            break;
            case Qt::Key_Space     :
                key[Interaction::SPACE] = false;
                Interaction::pause ^= true;
            break;
            case Qt::Key_Backspace :
                key[Interaction::BACKSPACE] = false;
                Interaction::rewind ^= true;
            break;
            case Qt::Key_Escape    :
                key[Interaction::ESCAPE] = false;
                QApplication::quit();
            break;
            case Qt::Key_P         :
                key[Interaction::PARALLEL] = false;
                Settings::PARALLEL_OMP ^= true;
            break;
            case Qt::Key_R         :
                key[Interaction::RENDER] = true;
                if (Interaction::pause)
                    render();
                key[Interaction::RENDER] = false;
            break;
            case Qt::Key_C         :
                key[Interaction::CONTROL] = false;
                if      (Settings::CONTROL_MODE == Interaction::ONE_DRAG)
                    Settings::CONTROL_MODE = Interaction::LOCAL_DRAG;
                else if (Settings::CONTROL_MODE == Interaction::LOCAL_DRAG)
                    Settings::CONTROL_MODE = Interaction::FORCE_DRAG;
                else if (Settings::CONTROL_MODE == Interaction::FORCE_DRAG)
                    Settings::CONTROL_MODE = Interaction::ONE_DRAG;
            break;
        }
    }
}

void SimulationWindow::mouseMoveEvent(QMouseEvent* event)
{
    mousePoint = event->pos();
    normalizedX =    (mousePoint.x() / (float) width())  * 2 - 1;
    normalizedY = - ((mousePoint.y() / (float) height()) * 2 - 1);
    dy = ( width() / 2) - mousePoint.x();
    dx = (height() / 2) - mousePoint.y();

    if (! key[Interaction::CTRL])
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
        if (key[Interaction::LMB])
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

    if (key[Interaction::CTRL])
    {
        switch(event->button())
        {
            case Qt::LeftButton :
                key[Interaction::LMB] = true;
                Interaction::handleTouchPress(normalizedX, normalizedY);
            break;
            case Qt::RightButton :
                key[Interaction::RMB] = true;
            break;
            default : break;
        }
    }
}
void SimulationWindow::mouseReleaseEvent(QMouseEvent* event)
{
    if (key[Interaction::CTRL])
    {
        switch(event->button())
        {
            case Qt::LeftButton :
                key[Interaction::LMB] = false;
//                DebugHelper::showVariable("normx", normalizedX);
//                DebugHelper::showVariable("normy", normalizedY);
//                DebugHelper::showVariables("aaaa", 12, 13, 15);
                Interaction::handleTouchDrop();
            break;
            case Qt::RightButton :
                key[Interaction::RMB] = false;
            break;
            default : break;
        }
    }
}

void SimulationWindow::w()
{
    Matrices::camTX += moveSpeed * sin(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * fabs(dt);
    Matrices::camTY -= moveSpeed * sin(Matrices::camRX * degToRad) * fabs(dt);
    Matrices::camTZ -= moveSpeed * cos(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * fabs(dt);
}
void SimulationWindow::s()
{
    Matrices::camTX -= moveSpeed * sin(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * fabs(dt);
    Matrices::camTY += moveSpeed * sin(Matrices::camRX * degToRad) * fabs(dt);
    Matrices::camTZ += moveSpeed * cos(Matrices::camRY * degToRad)
                                 * cos(Matrices::camRX * degToRad) * fabs(dt);
}
void SimulationWindow::a()
{
    Matrices::camTX -= moveSpeed * cos(Matrices::camRY * degToRad) * fabs(dt);
    Matrices::camTZ -= moveSpeed * sin(Matrices::camRY * degToRad) * fabs(dt);
}
void SimulationWindow::d()
{
    Matrices::camTX += moveSpeed * cos(Matrices::camRY * degToRad) * fabs(dt);
    Matrices::camTZ += moveSpeed * sin(Matrices::camRY * degToRad) * fabs(dt);
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
