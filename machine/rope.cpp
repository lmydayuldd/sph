#include "rope.h"

#include "physics/vector.h"

using namespace std;

Rope::~Rope()
{
}

Rope::Rope(
        const Vector& start, const Vector& end,
        int knots, float ks, float d, float kd, int strength
)
    : flow(vector<Particle>(knots))
{
    if      (strength == 0)     strength = 1;
    else if (strength >= knots) strength = knots - 1;

    for (int j = 1; j <= strength; ++j) {
        for (int i = j; i < knots; ++i)
            flow[i - j].springify(&flow[i], ks / j, d * j, kd / j);
        for (int i = j; i < knots; ++i)
            flow[i].springify(&flow[i - j], ks / j, d * j, kd / j);
    }

    /*for (int i = 1; i < knots; ++i) {
                   flow[i-1).springify(flow[i], ks, d, kd);
        if (i > 1) flow[i-2].springify(flow[i], ks/2, d*2, kd/2);
    }*/

    double nx = start.x;
    double ny = start.y;
    double nz = start.z;
    flow[0].r       = new Vector(start);
    flow[knots-1].r = new Vector(end);
    for (int i = 1; i < knots - 1; ++i) {
        nx += (end.x - start.x) / knots;
        ny += (end.y - start.y) / knots;
        nz += (end.z - start.z) / knots;
        flow[i].r->x = nx;
        flow[i].r->y = ny;
        flow[i].r->z = nz;
    }
    flow[0].stationary            = true;
    flow[knots - 1].stationary = true;

    float color[4] = {1.0f, 1.0f, 0.0f, 0.5f};
    for (unsigned int i = 0; i < flow.size(); ++i) {
        flow[i].v = new Vector();
        flow[i].recolor(color);
    }
}

void Rope::paint() {

}

void Rope::collide(Particle* p2) {}
