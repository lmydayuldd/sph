#include "window/simulation_window.h"

#include <QApplication>
#include <QDesktopServices>
#include <QDir>
#include <QScreen>
#include <QString>
#include <QKeyEvent>
#include <QMouseEvent>

#include <iostream>

#include <omp.h>
#include "mpi.h"
#ifdef COMPILER_MSVC
#include <cuda.h>
#endif

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
#include "util/strings.h"
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

////////////////////////////////////////////////////////////////////////////////

#ifdef COMPILER_MSVC
    int deviceCount = 0;
    int cudaDevice = 0;
    char cudaDeviceName[100];
    cuInit(0);
    cuDeviceGetCount(&deviceCount);
    cuDeviceGet(&cudaDevice, 0);
    cuDeviceGetName(cudaDeviceName, 100, cudaDevice);
    qDebug() << "Number of devices: " << deviceCount;
    qDebug() << "Device name:" << cudaDeviceName;
#endif
}

SimulationWindow::~SimulationWindow()
{
    std::cout << "Exiting the program." << std::flush;
}

void SimulationWindow::prepareSimulation()
{
    omp_set_num_threads(Settings::PARALLEL_OMP_THREADS);

    Strings::init(); // TODO // called Strings but reponsible for dirs also...

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

    Machine::machines.push_back(new Walls(Settings::ARENA_DIAMETER / 2));
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
}

void SimulationWindow::initialize()
{
    glEnable(GL_CULL_FACE);
    glEnable(GL_DEPTH_TEST);

    const qreal retinaScale = devicePixelRatio();
    glViewport(0, 0, width() * retinaScale, height() * retinaScale);
    refreshRate = screen()->refreshRate(); // frame // fps // frequency // period
    QCursor::setPos(geometry().x() + width()/2, geometry().y() + height()/2);

//    Matrices::viewMatrix.translate(QVector3D(10, 10, 10));
    Matrices::camTZ = 12;
    Matrices::camRY = 0;
    Matrices::projectionMatrix.setToIdentity();
    Matrices::projectionMatrix.perspective(90.f, width()/height(), 0.1f, 100.f);

    Shader::currentShader = new Shader();
    Computer::currentComputer = new Computer();

    prepareSimulation();
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

    if (! Interaction::pause || SimulationWindow::key[RENDER]) {
        std::cout << round(dt/1000000.0) << "ms <- Frame time." << std::endl;
        std::cout << "~" << round(1000000000/dt) << " FPS" << std::endl;
        std::cout << "-- Frame " << frame-1 << " end ---------------------------" << std::endl << std::endl;
        std::cout << "-- Frame " << frame << " start --------------------------" << std::endl;
    }

    #pragma omp parallel if(Settings::PARALLEL_OMP)
    {
        #pragma omp master
        if (! Interaction::pause || SimulationWindow::key[RENDER])
            std::cout << omp_get_num_threads() << " threads." << std::endl;
    }

    interact(); //////////////////////////////////////////////
    if (! Interaction::pause || SimulationWindow::key[RENDER])
    {
        for (unsigned i = 0; i < Settings::ITERATIONS_PER_FRAME; ++i)
        {
            Computer::currentComputer->loop();
        }
    }

    Timer timer = Timer();
    long long int formerTime = 0;
    formerTime = timer.diff();

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
    Shader::currentShader->program->release();

    if (! Interaction::pause || SimulationWindow::key[RENDER])
        std::cout << (timer.diff() - formerTime)/1000000
                  << "ms <- Frame drawing time." << std::endl;

    if (! Interaction::pause)
    {
        QPixmap pixMap = screen()->grabWindow(0, x(), y(), width(), height());
        QImage img = pixMap.toImage();
        QString dst = Strings::DIR_FRAMES + QString("frame_%1.bmp").arg(frame);
        img.save(dst);
    }

    if (! Interaction::pause || SimulationWindow::key[RENDER])
    {
        ++frame;
    }
}

void SimulationWindow::interact()
{
    if (key[W])    w();
    if (key[S])    s();
    if (key[A])    a();
    if (key[D])    d();
    if (key[DUCK]) l();
    Interaction::holdPressedParticle();
}

void SimulationWindow::keyPressEvent(QKeyEvent *e)
{
    switch (e->key())
    {
        case Qt::Key_W         : key[W]         = true; break;
        case Qt::Key_Up        : key[W]         = true; break;
        case Qt::Key_S         : key[S]         = true; break;
        case Qt::Key_Down      : key[S]         = true; break;
        case Qt::Key_A         : key[A]         = true; break;
        case Qt::Key_Left      : key[A]         = true; break;
        case Qt::Key_D         : key[D]         = true; break;
        case Qt::Key_Right     : key[D]         = true; break;
        case Qt::Key_V         : key[DUCK]      = true; break;
        case Qt::Key_Control   : key[CTRL]      = true; break;
        case Qt::Key_Space     : key[SPACE]     = true; break;
        case Qt::Key_Backspace : key[BACKSPACE] = true; break;
        case Qt::Key_Escape    : key[ESCAPE]    = true; break;
        case Qt::Key_P         : key[PARALLEL]  = true; break;
//        case Qt::Key_R         : key[RENDER]    = true; break;
        case Qt::Key_C         : key[CONTROL]   = true; break;
    }
}

void SimulationWindow::keyReleaseEvent(QKeyEvent *e)
{
    if (! e->isAutoRepeat())
    {
        switch (e->key())
        {
            case Qt::Key_W         : key[W]     = false; break;
            case Qt::Key_S         : key[S]     = false; break;
            case Qt::Key_A         : key[A]     = false; break;
            case Qt::Key_D         : key[D]     = false; break;
            case Qt::Key_V         : key[DUCK]  = false; break;
            case Qt::Key_Control   :
                key[CTRL] = false;
                Interaction::handleTouchDrop();
            break;
            case Qt::Key_Space     :
                key[SPACE] = false;
                Interaction::pause ^= true;
            break;
            case Qt::Key_Backspace :
                key[BACKSPACE] = false;
                Interaction::rewind ^= true;
            break;
            case Qt::Key_Escape    :
                key[ESCAPE] = false;
                QApplication::quit();
                saveVideo();
                QDesktopServices::openUrl(Strings::DIR_FRAMES);
            break;
            case Qt::Key_P         :
                key[PARALLEL] = false;
                Settings::PARALLEL_OMP ^= true;
            break;
            case Qt::Key_R         :
                key[RENDER] = true;
                if (Interaction::pause)
                {
                    render();
                }
                key[RENDER] = false;
            break;
            case Qt::Key_C         :
                key[CONTROL] = false;
                switch (Settings::CONTROL_MODE) {
                    case ONE_DRAG   : Settings::CONTROL_MODE = LOCAL_DRAG; break;
                    case LOCAL_DRAG : Settings::CONTROL_MODE = FORCE_DRAG; break;
                    case FORCE_DRAG : Settings::CONTROL_MODE = ONE_DRAG;   break;
                }
            break;
        }
    }
}

void SimulationWindow::mouseMoveEvent(QMouseEvent *event)
{
    mousePoint = event->pos();
    normalizedX =    (mousePoint.x() / (float) width())  * 2 - 1;
    normalizedY = - ((mousePoint.y() / (float) height()) * 2 - 1);
    dy = ( width() / 2) - mousePoint.x();
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

void SimulationWindow::mousePressEvent(QMouseEvent *event)
{
    mousePoint = event->pos();
    normalizedX =    (mousePoint.x() / (float) width())  * 2 - 1;
    normalizedY = - ((mousePoint.y() / (float) height()) * 2 - 1);

    if (key[CTRL])
    {
        switch (event->button())
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
void SimulationWindow::mouseReleaseEvent(QMouseEvent *event)
{
    if (key[CTRL])
    {
        switch (event->button())
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

void SimulationWindow::saveVideo()
{
    QString path = Strings::DIR_FRAMES;
    QDir dir(path);
    dir.setFilter(QDir::AllEntries | QDir::NoDotAndDotDot);
    unsigned files = dir.count();
    for (unsigned i = 0; i < files; ++i)
    {
        QString frame_path = Strings::DIR_FRAMES + QString("frame_%1.bmp").arg(i);
    }
}
