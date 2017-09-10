#include "shape/line.h"

#include "gl/form.h"

#include <QOpenGLFunctions>

using namespace std;

Line::Line(vector<float> ends, vector<float> color)
{
    posCoordsPerVertex = ends.size() / 2;
    if (posCoordsPerVertex == 2) {
        vertices = vector<float> {
            ends[0], ends[1], color[0], color[1], color[2],
            ends[2], ends[3], color[0], (float) fmod(color[1] + 0.5, 1), color[2]
        };
    }
    else if (posCoordsPerVertex == 3) {
        vertices = vector<float> {
            ends[0], ends[1], ends[2], color[0], color[1], color[2],
            ends[3], ends[4], ends[5], color[0], (float) fmod(color[1] + 0.5, 1), color[2]
        };
    }
}

void Line::makeForm()
{
    form = new Form(
        vertices, posCoordsPerVertex, GL_LINES,
        vector<float>(), 3,
        vector<float>(), 0, 0,
        LINE
    );
}

void Line::move(float x1, float y1, float x2, float y2)
{
    form->posCoords[0] = x1;
    form->posCoords[1] = y1;
    form->posCoords[5] = x2;
    form->posCoords[6] = y2;
    form->move(); // initialize vertex byte buffer for shape coordinates
}
