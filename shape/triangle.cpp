#include "shape/triangle.h"

#include <cmath>

#include <QOpenGLFunctions>

#include <gl/form.h>

Triangle::Triangle(float side)
    : side(side)
{
    float color[] = {0.5, 0.0, 0.0};
    float a[] = { 1, -1, 0};
    float b[] = { 0,  1, 0};
    float c[] = {-1, -1, 0};
    vertices.insert(vertices.end(), &a[0], &a[sizeof(a) / sizeof(float)]);
    vertices.insert(vertices.end(), &color[0], &color[sizeof(color) / sizeof(float)]);
    vertices.insert(vertices.end(), &b[0], &b[sizeof(b) / sizeof(float)]);
    vertices.insert(vertices.end(), &color[0], &color[sizeof(color) / sizeof(float)]);
    vertices.insert(vertices.end(), &c[0], &c[sizeof(c) / sizeof(float)]);
    vertices.insert(vertices.end(), &color[0], &color[sizeof(color) / sizeof(float)]);
}

void Triangle::makeForm()
{
    form = new Form(
        vertices, 3, GL_TRIANGLES,
        std::vector<float>(), 3,
        std::vector<float>(), 0, 0,
        TRIANGLE
    );
}
