#include "shape/rectangle.h"

#include <cmath>

#include "gl/form.h"
#include "shape/line.h"

#include "util/settings.h"
#include "physics/grid.h"

using namespace std;
using namespace shapeSpace;

Rectangle::Rectangle(float c[]/*oords*//*x1 x2 x3 y1 y2*/, float color[])
{
    vertices = vector<float> {
        c[0],  c[1],  c[2],  color[0], (float) fmod(color[1] + 0.5f, 1.0f), color[2],
        c[3],  c[4],  c[5],  color[0], color[1], color[2],
        c[6],  c[7],  c[8],  color[0], color[1], color[2],
        c[9],  c[10], c[11], color[0], color[1], color[2],
        c[12], c[13], c[14], color[0], color[1], color[2],
        c[3],  c[4],  c[5],  color[0], color[1], color[2]
    };
    makeForm();
}

Rectangle::Rectangle(float lim, float color[])
{
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

    // TODO
    //   this should be placed elsewhere
    //   it doesn't concern rectangle
    //   it doesn't even concert arena
    //   it concerns the particle neighbourhood grid
    for (unsigned i = 0; i <= Grid::cell_count; i += 1)
    {
        vector<float> v1 = { lim                                  ,
                            -lim + (float) (i*Grid::cell_diameter),
                            -lim                                  ,
                            -lim + (float) (i*Grid::cell_diameter)};
        vector<float> v2 = {-lim + (float) (i*Grid::cell_diameter),
                             lim                                  ,
                            -lim + (float) (i*Grid::cell_diameter),
                            -lim                                  };
        Line l1 = Line(v1, vector<float> {color[0], color[1], color[2]});
        Line l2 = Line(v2, vector<float> {color[0], color[1], color[2]});
        vertices.insert(vertices.end(), l1.vertices.begin(), l1.vertices.end());
        vertices.insert(vertices.end(), l2.vertices.begin(), l2.vertices.end());
    }

    makeForm();
}

void Rectangle::makeForm()
{
    posCoordsPerVertex = 2;//3
    clrCoordsPerVertex = 3;
    form = new Form(
        vertices, posCoordsPerVertex, GL_LINES,//GL_TRIANGLE_FAN
        vector<float>(), clrCoordsPerVertex,
        vector<float>(), 0, 0,
        RECTANGLE
    );
}

void Rectangle::move(
    float x1, float y1,
    float x2, float y2,
    float x3, float y3,
    float x4, float y4,
    float x5, float y5
) {
    form->posCoords[0]  = x1;
    form->posCoords[1]  = y1;
    form->posCoords[6]  = x2;
    form->posCoords[7]  = y2;
    form->posCoords[12] = x3;
    form->posCoords[13] = y3;
    form->posCoords[18] = x4;
    form->posCoords[19] = y4;
    form->posCoords[24] = x5;
    form->posCoords[25] = y5;
    form->posCoords[30] = x2;
    form->posCoords[31] = y2;
    form->move(); // initialize vertex byte buffer for shape coordinates
}
