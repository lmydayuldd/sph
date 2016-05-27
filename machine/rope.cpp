#include "rope.h"

#include "physics/vector.h"

Rope::~Rope()
{
    delete flow;
}

Rope::Rope(Vector* start, Vector* end, int knots, float ks, float d, float kd, int strength) {
    start = start;
    end = end;
    knots = knots;
    ks = ks;
    d = d;
    kd = kd;
    strength = strength;
//    flow = Flow(knots, 0, 0, 0);

//    if (strength == 0)     strength = 1;
//    if (strength >= knots) strength = knots - 1;
//    for (int j = 1; j <= strength; ++j) {
//        for (int i = j; i < knots; ++i)
//            flow.particles.get(i - j).springify(flow.particles.get(i), ks / j, d * j, kd / j);
//        for (int i = j; i < knots; ++i)
//            flow.particles.get(i).springify(flow.particles.get(i - j), ks / j, d * j, kd / j);
//    }

//    /*for (int i = 1; i < knots; ++i) {
//                   flow.particles.get(i-1).springify(flow.particles.get(i), ks, d, kd);
//        if (i > 1) flow.particles.get(i-2).springify(flow.particles.get(i), ks/2, d*2, kd/2);
//    }*/

//    double nx = start.x;
//    double ny = start.y;
//    double nz = start.z;
//    flow.particles[0].position       = Vector(start);
//    flow.particles[knots-1].position = Vector(end);
//    for (int i = 1; i < knots - 1; ++i) {
//        nx += (end.v[0] - start.v[0]) / knots;
//        ny += (end.v[1] - start.v[1]) / knots;
//        nz += (end.v[2] - start.v[2]) / knots;
//        flow.particles.get(i).position.v[0] = nx;
//        flow.particles.get(i).position.v[1] = ny;
//        flow.particles.get(i).position.v[2] = nz;
//    }
//    flow.particles.get(0).stationary         = true;
//    flow.particles.get(knots - 1).stationary = true;

//    float[] color = new float[] { 1.0f, 1.0f, 0.0f, 0.5f };
//    for (int i = 0; i < Flow.flows[0].particles.size(); ++i) {
//        p.v = Vector();
//        p.recolor( color );
//    }
}

void Rope::paint() {

}
