#include "dot.h"

#include <gl/form.h>

using namespace std;

Dot::~Dot()
{
    delete form;
}

Dot::Dot()
    : color {0, 0, 0}
{
}

Dot::Dot(float color[])
{
    if (color == nullptr) vertices = vector<float> {0, 0, 0, 0, 0};
    else                  vertices = vector<float> {0, 0, color[0], color[1], color[2]};
}

void Dot::makeForm() {
//    vertices[0] = position[0];
//    vertices[1] = position[1];
    form = new Form(
        vertices, 2, GL_POINTS,
        vector<float>(), 3,
        vector<float>(), 0, 0,
        DOT
    );
}

//void Dot::move(float x, float y) {
//    position[0] = x;
//    position[1] = y;
//    form->posCoords[0] = x;
//    form->posCoords[4] = y;
//    form->move();
//}
