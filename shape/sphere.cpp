#include "shape/sphere.h"

#include <cmath>

#include <QOpenGLFunctions>

#include "gl/form.h"
#include "util/settings.h"

using namespace std;

Sphere::Sphere(double radius, float color[])
    : center(vector<float> {0, 0, 0}),
      smoothness(Settings::SPHERE_DETAIL),
      radius(radius)
{
    float step = M_PI / smoothness;
    color[1] = 0.0f;
    for (float i = 0.0f; i < M_PI; i += step) {
        color[1] = color[1] + step / M_PI;
        if      (color[1] >= 1.0f) color[1] -= fmod(color[1], 1);
        else if (color[1] <= 0.0f) color[1] += fmod(fabs(color[1]), 1);
        for (float j = 0.0f; j < 2*M_PI; j += step) {
            vertices.push_back(radius * cos(j) * sin(i) + center[0]);
            vertices.push_back(radius * sin(j) * sin(i) + center[1]);
            vertices.push_back(radius * cos(i));
//            vertices.insert(vertices.end(), &color[0], &color[sizeof(color) / sizeof(float)]);
            vertices.insert(vertices.end(), &color[0], &color[3]);

            vertices.push_back(radius * cos(j) * sin(i + step) + center[0]);
            vertices.push_back(radius * sin(j) * sin(i + step) + center[1]);
            vertices.push_back(radius * cos(i + step));
//            vertices.insert(vertices.end(), &color[0], &color[sizeof(color) / sizeof(float)]);
            vertices.insert(vertices.end(), &color[0], &color[3]);
        }
    }
    makeForm();
}

void Sphere::makeForm()
{
    form = new Form(
        vertices, 3, GL_TRIANGLE_STRIP,
        vector<float>(), 3,
        vector<float>(), 0, 0,
        SPHERE
    );
}
