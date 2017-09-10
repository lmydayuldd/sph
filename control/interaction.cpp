#include "control/interaction.h"

#include <QApplication>
#include <QDesktopServices>

#include <typeinfo>

#include "gl/matrices.h"
#include "machine/particle.h"
#include "physics/computer.h"
#include "physics/geometry.h"
#include "util/constants.h"
#include "util/operations.h"
#include "util/settings.h"
#include "util/strings.h"
#include "window/simulation_window.h"

Particle *Interaction::pressedParticle = nullptr;
Vector *Interaction::lockedPressedParticlePosition = nullptr;
Vector *Interaction::pressedPoint = nullptr;
QPoint *Interaction::mousePoint = nullptr;
bool Interaction::pause = true;
bool Interaction::rewind = false;
bool Interaction::key[] = {0};
float Interaction::t = 1.0f;
int Interaction::dx = 0;
int Interaction::dy = 0;
float Interaction::normalizedX = 0.;
float Interaction::normalizedY = 0.;
float Interaction::cursorX = 0.;
float Interaction::cursorY = 0.;
float Interaction::moveSpeed = 0.000000005f;//10.f;
float Interaction::mouseSpeed = 0.3f;
float Interaction::previousX = 0.;
float Interaction::previousY = 0.;
float Interaction::previousDistance = 0.;
std::string Interaction::previousAction;

extern void GPUCollideFrees();

void Interaction::handleTouchPress(float normalizedX, float normalizedY)
{
    Geometry::Ray ray = convertNormalized2DPointToRay(normalizedX, normalizedY);
    //if (Computer::currentComputer != nullptr) {
//        for (Flow f : Flow.flows) for (Particle p : f.particles) {
//            Sphere particleBoundingSphere = new Sphere(new Vector(p.r), p.radius);
//            if (intersects(particleBoundingSphere, ray)) {
//                pressedParticle = p;
//                break;
//            }
//        }

        double minProximity = std::numeric_limits<double>::infinity();
        Particle *closest(nullptr);
        for (unsigned i = 0; i < Particle::flows.size(); ++i)
        {
            for (Particle *p : Particle::flows[i])
            {
                Geometry::Sphere particleBoundingSphere
                        = Geometry::Sphere(*p->r, p->radius);
                double distance
                        = distanceBetween(particleBoundingSphere.center, ray);
                if (distance < minProximity)
                {
                    minProximity = distance;
                    closest = p;
                }
            }
        }
        pressedParticle = closest;
    //}

//    if (typeid(Computer::currentComputer) == typeid(Wave2DComputer)) {
//        double minProximity = std::numeric_limits<double>::infinity();
//        vector<Shape> shapes = Wave2DComputer.landschaft.shapes;
//        int x = 0, y = 0;
//        for (auto s = begin(shapes); s != end(shapes); ++s) {
//            for (unsigned j = 0; j < (*s).posCoords.size(); j += 24) { ////////////////////////////////////////
//                Sphere quadrantBoundingSphere = Sphere(
//                    Vector((*s).posCoords[j], (*s).posCoords[j+1], (*s).posCoords[j+2]),
//                    Computer::currentComputer->dx
//                );
//                double distance = distanceBetween(quadrantBoundingSphere.center, ray);
//                if (distance < minProximity) {
//                    minProximity = distance;
//                    x = s - begin(shapes);
//                    y = j;
//                }
//            }
//        }
//        Computer::currentComputer->flagUp(x, y / 12, Wave2DComputer.CELL_BND);
//    }
}

void Interaction::handleTouchDrag(float normalizedX, float normalizedY)
{
    Geometry::Ray ray = convertNormalized2DPointToRay(normalizedX, normalizedY);
    //if (typeid(Computer::currentComputer) == typeid(ParticleComputer)) {
        if (pressedParticle != nullptr)
        {
            // Define plane parallel to default view plane and crossing Particle
            Vector pressedParticlePosition(*pressedParticle->r);
            Vector straightVector(0, 0, 1);
            Geometry::Plane plane
                   = Geometry::Plane(pressedParticlePosition, straightVector);

            // Find out where the touched point intersects the
            // aforementioned plane. We'll move the particle along it.
            Vector touchedPoint = intersectionPoint(ray, plane);

            if       (Settings::CONTROL_MODE == ONE_DRAG) {
                lockedPressedParticlePosition
                    = new Vector(touchedPoint.x, touchedPoint.y, pressedParticle->r->z);
                holdPressedParticle();
            }
            else if (Settings::CONTROL_MODE == LOCAL_DRAG) {
                lockedPressedParticlePosition
                    = new Vector(touchedPoint.x, touchedPoint.y, pressedParticle->r->z);
                for (unsigned i = 0; i < Particle::flows[0].size(); ++i)
                {
                    if (Particle::flows[0][i]->cell[0] == pressedParticle->cell[0]
                     && Particle::flows[0][i]->cell[1] == pressedParticle->cell[1]
                     && Particle::flows[0][i]->cell[2] == pressedParticle->cell[2]) {
                        *Particle::flows[0][i]->r
                            = *lockedPressedParticlePosition - *Particle::flows[0][i]->r;
                    }
                }
            }
            else if (Settings::CONTROL_MODE == FORCE_DRAG) {
                Particle *n(nullptr);
                if (pressedParticle->neighbours != nullptr) // TODO, in particle, neigh vector can't ever be null, but rather empty
                {
                    for (unsigned i = 0; i < pressedParticle->neighbours->size(); ++i)
                    {
                        n = pressedParticle->neighbours->at(i);
                        *n->v += (*n->r - *pressedParticle->r).normal() * 0.05;
                    }
                }
            }
        }
    //}
}

void Interaction::holdPressedParticle()
{
    if (pressedParticle != nullptr)
    {
        if (lockedPressedParticlePosition != nullptr)
        {
            *pressedParticle->r = *lockedPressedParticlePosition;
            *pressedParticle->v = Vector();
        }
    }
}

void Interaction::handleTouchDrop()
{
    pressedParticle = nullptr;
    lockedPressedParticlePosition = nullptr;
}

void Interaction::keyPress(QKeyEvent *e)
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
        case Qt::Key_G         : key[ROTATE]    = true; break;
    }
}

void Interaction::keyRelease(QKeyEvent *e)
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
                GPUCollideFrees();
                QApplication::quit();
                if (! Settings::NO_SCREENS_NO_VIDEO) {
                    SimulationWindow::saveVideo();
                    QDesktopServices::openUrl(Strings::DIR_FRAMES);
                }
            break;
            case Qt::Key_P         :
                key[PARALLEL] = false;
                Settings::PARALLEL_OMP ^= true;
            break;
            case Qt::Key_R         :
                key[RENDER] = true;
                if (Interaction::pause)
                {
                    SimulationWindow::simWin->render();
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
            case Qt::Key_G         :
                key[ROTATE] = false;
                Settings::WORLD_ROTATION += 10.;
                Settings::WORLD_ROTATION = fmod(Settings::WORLD_ROTATION, 360.);
            break;
        }
    }
}

void Interaction::mouseMove(QMouseEvent *event)
{
    mousePoint = &event->pos();
    normalizedX =    (mousePoint->x() / (float) SimulationWindow::simWin->width())  * 2 - 1;
    normalizedY = - ((mousePoint->y() / (float) SimulationWindow::simWin->height()) * 2 - 1);
    dy = ( SimulationWindow::simWin->width() / 2) - mousePoint->x();
    dx = (SimulationWindow::simWin->height() / 2) - mousePoint->y();

    if (! key[CTRL])
    {
        Matrices::camRX -= dx * mouseSpeed;
        Matrices::camRY -= dy * mouseSpeed;
        if (Matrices::camRX >  90) Matrices::camRX =  90;
        if (Matrices::camRX < -90) Matrices::camRX = -90;

        QCursor::setPos(SimulationWindow::simWin->geometry().x() + SimulationWindow::simWin->width()/2,
                        SimulationWindow::simWin->geometry().y() + SimulationWindow::simWin->height()/2);
    }
    else
    {
        if (key[LMB])
        {
            Interaction::handleTouchDrag(normalizedX, normalizedY);
        }
    }
}

void Interaction::mousePress(QMouseEvent *event)
{
    mousePoint = &event->pos();
    normalizedX =    (mousePoint->x() / (float) SimulationWindow::simWin->width())  * 2 - 1;
    normalizedY = - ((mousePoint->y() / (float) SimulationWindow::simWin->height()) * 2 - 1);

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
void Interaction::mouseRelease(QMouseEvent *event)
{
    if (key[CTRL])
    {
        switch (event->button())
        {
            case Qt::LeftButton :
                key[LMB] = false;
                Interaction::handleTouchDrop();
            break;
            case Qt::RightButton :
                key[RMB] = false;
            break;
            default : break;
        }
    }
}

void Interaction::interact()
{
    if (Interaction::key[W])    w();
    if (Interaction::key[S])    s();
    if (Interaction::key[A])    a();
    if (Interaction::key[D])    d();
    if (Interaction::key[DUCK]) l();
    Interaction::holdPressedParticle();
}

void Interaction::w()
{
    Matrices::camTX += Interaction::moveSpeed * sin(Matrices::camRY * degToRad)
                                              * cos(Matrices::camRX * degToRad)
                                              * fabs(SimulationWindow::frame_dt);
    Matrices::camTY -= Interaction::moveSpeed * sin(Matrices::camRX * degToRad)
                                              * fabs(SimulationWindow::frame_dt);
    Matrices::camTZ -= Interaction::moveSpeed * cos(Matrices::camRY * degToRad)
                                              * cos(Matrices::camRX * degToRad)
                                              * fabs(SimulationWindow::frame_dt);
}
void Interaction::s()
{
    Matrices::camTX -= Interaction::moveSpeed * sin(Matrices::camRY * degToRad)
                                              * cos(Matrices::camRX * degToRad)
                                              * fabs(SimulationWindow::frame_dt);
    Matrices::camTY += Interaction::moveSpeed * sin(Matrices::camRX * degToRad)
                                              * fabs(SimulationWindow::frame_dt);
    Matrices::camTZ += Interaction::moveSpeed * cos(Matrices::camRY * degToRad)
                                              * cos(Matrices::camRX * degToRad)
                                              * fabs(SimulationWindow::frame_dt);
}
void Interaction::a()
{
    Matrices::camTX -= Interaction::moveSpeed * cos(Matrices::camRY * degToRad) * fabs(SimulationWindow::frame_dt);
    Matrices::camTZ -= Interaction::moveSpeed * sin(Matrices::camRY * degToRad) * fabs(SimulationWindow::frame_dt);
}
void Interaction::d()
{
    Matrices::camTX += Interaction::moveSpeed * cos(Matrices::camRY * degToRad) * fabs(SimulationWindow::frame_dt);
    Matrices::camTZ += Interaction::moveSpeed * sin(Matrices::camRY * degToRad) * fabs(SimulationWindow::frame_dt);
}
void Interaction::h()
{
    Matrices::cam[1] += t;
    Matrices::cam[4] += t;
}
void Interaction::l()
{
    Matrices::cam[1] -= t;
    Matrices::cam[4] -= t;
}
void Interaction::r()
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
