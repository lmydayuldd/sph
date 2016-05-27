#include "machine/walls.h"

#include <QOpenGLFunctions>

#include "gl/form.h"
#include "gl/matrices.h"
#include "machine/particle.h"
#include "physics/vector.h"
#include "shape/rectangle.h"
#include "shape/line.h"
#include "util/settings.h"

using namespace std;

Walls::Walls(float lim)
    : lim(lim),
      damping(Settings::DAMPENING)
{
    linkView(RECTANGLE);
}

void Walls::createView()
{
//    float color[3] = {0, 0.5, 0};
//    Rectangle rectangle(lim, color);
//    this->form = rectangle.form;
    vector<float> vertices;
    float color[3] = {0, 0.5, 0};
    vector<float> v1 = { lim,  lim, -lim,  lim};
    vector<float> v2 = {-lim,  lim, -lim, -lim};
    vector<float> v3 = {-lim, -lim,  lim, -lim};
    vector<float> v4 = { lim, -lim,  lim,  lim};
    Line l1 = Line(v1, vector<float> {color[0], color[1], color[2]});
    Line l2 = Line(v2, vector<float> {color[0], color[1], color[2]});
    Line l3 = Line(v3, vector<float> {color[0], color[1], color[2]});
    Line l4 = Line(v4, vector<float> {color[0], color[1], color[2]});
    vertices.insert(vertices.end(), l1.vertices.begin(), l1.vertices.end());
    vertices.insert(vertices.end(), l2.vertices.begin(), l2.vertices.end());
    vertices.insert(vertices.end(), l3.vertices.begin(), l3.vertices.end());
    vertices.insert(vertices.end(), l4.vertices.begin(), l4.vertices.end());
    form = new Form(
        vertices, 2, GL_LINES,
        vector<float>(), 3,
        vector<float>(), 0, 0,
        RECTANGLE
    );
}

void Walls::setModelMatrix()
{
    Matrices::modelMatrix.setToIdentity();
}

void Walls::paint()
{
    setModelMatrix();

    Machine::paint();
}

void Walls::collide(Particle* p2) {
    if ( p2->r->x < -lim || p2->r->x > lim ||
         p2->r->y < -lim || p2->r->y > lim ) {
        if      (p2->r->x < -lim) { p2->r->x = -lim; p2->v->x *= -1 * (1. - damping); }
        else if (p2->r->x >  lim) { p2->r->x =  lim; p2->v->x *= -1 * (1. - damping); }
        else if (p2->r->y < -lim) { p2->r->y = -lim; p2->v->y *= -1 * (1. - damping); }
        else if (p2->r->y >  lim) { p2->r->y =  lim; p2->v->y *= -1 * (1. - damping); }
    }
}
