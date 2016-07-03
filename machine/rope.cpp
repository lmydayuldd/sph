#include "rope.h"

#include "physics/vector.h"

using namespace std;

Rope::~Rope()
{
}

Rope::Rope(
    const Vector& start, const Vector& end,
    int knots, int strength,
    float ks, float d, float kd)
{
    if      (strength == 0)     strength = 1;
    else if (strength >= knots) strength = knots - 1;

    Particle::flows.push_back(vector<Particle*>(knots));
    flow = &Particle::flows.back();
    for (int i = 0; i < knots; ++i)
        (*flow)[i] = new Particle(Particle::flows.size() - 1);

    for (int j = 1; j <= strength; ++j) {
        for (int i = j; i < knots; ++i)
            (*flow)[i - j]->springify((*flow)[i], ks / j, d * j, kd / j);
        for (int i = j; i < knots; ++i)
            (*flow)[i]->springify((*flow)[i - j], ks / j, d * j, kd / j);
    }

    /*for (int i = 1; i < knots; ++i) {
                   flow[i-1).springify(flow[i], ks, d, kd);
        if (i > 1) flow[i-2].springify(flow[i], ks/2, d*2, kd/2);
    }*/

    Vector rn = start;
    (*flow)[0]->r       = new Vector(start);
    (*flow)[knots-1]->r = new Vector(end);
    for (int i = 1; i < knots - 1; ++i) {
        rn.x += (end.x - start.x) / knots;
        rn.y += (end.y - start.y) / knots;
        rn.z += (end.z - start.z) / knots;
        (*flow)[i]->r = new Vector(rn);
    }
    (*flow)[0]->stationary         = true;
    (*flow)[knots - 1]->stationary = true;

    float color[4] = {1, 1, 0, 0.5};
    for (unsigned int i = 0; i < (*flow).size(); ++i) {
        (*flow)[i]->v = new Vector();
        //flow[i].recolor(color);
    }
}

void Rope::paint() {
    // Using Shape iterator would require Shape object to be temporarily created
    // by non-parametred Shape().
    // It would then get destroyed, causing a ~Shape() call!
    for (unsigned int i = 0; i < (*flow).size(); ++i) {
        (*flow)[i]->paint();
    }
}

void Rope::collide(Particle* p2) {}
