#include "shape/arrow.h"

#include "gl/form.h"
#include "shape/line.h"

using namespace std;

Arrow::Arrow()
{
    vector<float> color = {1, 1, 0};
    vector<float> a = {0, 0,  0.0, 1.0};
    vector<float> b = {0, 1, -0.1, 0.9};
    vector<float> c = {0, 1,  0.1, 0.9};
    Line l1 = Line(a, color);
    Line l2 = Line(b, color);
    Line l3 = Line(c, color);
    vertices.insert(vertices.end(), l1.vertices.begin(), l1.vertices.end());
    vertices.insert(vertices.end(), l2.vertices.begin(), l2.vertices.end());
    vertices.insert(vertices.end(), l3.vertices.begin(), l3.vertices.end());
    makeForm();
}

void Arrow::makeForm()
{
    int posCoordsPerVertex = 2;
    int clrCoordsPerVertex = 3;
    form = new Form(
        vertices, posCoordsPerVertex, GL_LINES,
        vector<float>(), clrCoordsPerVertex,
        vector<float>(), 0, 0,
        ARROW
    );
}
