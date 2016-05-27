#include "machine/cloth.h"

#include "gl/matrices.h"
#include "machine/flow.h"
#include "machine/particle.h"
#include "physics/vector.h"
#include "shape/rectangle.h"

Cloth::~Cloth()
{
    delete flow;
}

Cloth::Cloth(
    Vector* start, Vector* end,
    int knots, float ks, float d, float kd//, int strength
)
{
    color[0] = 1.0f;
    color[1] = 1.0f;
    color[2] = 0.0f;

    this->knots = knots;
    flow = new Flow(knots * knots);//, 0, 0, 0);

    for (int i = 1; i < knots * knots; ++i) // hor-sing
        if (i % knots != 0)
            flow->particles[i].springifyMutual(&flow->particles[i - 1], ks, d, kd);
    for (int i = 2; i < knots * knots; ++i) // hor-dual
        if (i % knots > 1)
            flow->particles[i].springifyMutual(&flow->particles[i-2], ks/2, d*2, kd/2);

    for (int i = knots; i < knots * knots; ++i) // ver-sing
        flow->particles[i].springifyMutual(&flow->particles[i - knots], ks, d, kd);
    for (int i = 2 * knots; i < knots * knots; ++i) // ver-dual
        flow->particles[i].springifyMutual(&flow->particles[i-(2*knots)], ks/2, d*2, kd/2);

    for (int i = knots + 1; i < knots * knots; ++i) // skw-sing-upright
        if ((i-1)%knots != 0)
            flow->particles[i].springifyMutual(&flow->particles[i-knots-1], ks, d, kd);
    for (int i = knots * knots - knots; i > 0; --i) // skw-sing-upleft
        if (i%knots != 0)
            flow->particles[i].springifyMutual(&flow->particles[i+knots-1], ks, d, kd);

    double ny = start->y;
    double nz = start->z;
    for (int i = 0; i < knots; ++i) {
        ny += (end->y - start->y) / knots;
        nz += (end->z - start->z) / knots;
        double nx = start->x;
        for (int j = 0; j < knots; ++j) {
            nx += (end->x - start->x) / knots;
            flow->particles[i*knots + j].r->x = nx;
            flow->particles[i*knots + j].r->y = ny;
            flow->particles[i*knots + j].r->z = nz;
            if ((i == 0 && j == 0) || (i == 0 && j == knots-1) || (i == knots-1 && j == 0) || (i == knots-1 && j == knots-1)) {
                flow->particles[i*knots + j].stationary = true;
            }
        }
    }

    for (Particle p : flow->particles) p.v = new Vector();
    for (Particle p : flow->particles) p.recolor(color);
}

void Cloth::setModelMatrix()
{

}

void Cloth::paint()
{
//    float rr[15] = {0};
//    Rectangle r(rr, color);
//    if (this->forms.empty())
//        for (int i = 0; i < knots - 1; ++i)
//            for (int j = 0; j < knots - 1; ++j)
//                this->forms.push_back(r.shape);

//    for (int i = 0; i < knots - 1; ++i) {
//        for (int j = 0; j < knots - 1; ++j) {
//            Form currentForm = forms[i*(knots-1) + j];

//            currentForm.posCoords[6]  = (float) flow->particles[(i+1)*knots + j + 1].r.x;
//            currentForm.posCoords[7]  = (float) flow->particles[(i+1)*knots + j + 1].r.y;
//            currentForm.posCoords[8]  = (float) flow->particles[(i+1)*knots + j + 1].r.z;

//            currentForm.posCoords[24] = (float) flow->particles[i*knots + j + 1].r.x;
//            currentForm.posCoords[25] = (float) flow->particles[i*knots + j + 1].r.y;
//            currentForm.posCoords[26] = (float) flow->particles[i*knots + j + 1].r.z;

//            currentForm.posCoords[18] = (float) flow->particles[i*knots + j].r.x;
//            currentForm.posCoords[19] = (float) flow->particles[i*knots + j].r.y;
//            currentForm.posCoords[20] = (float) flow->particles[i*knots + j].r.z;

//            currentForm.posCoords[12] = (float) flow->particles[(i+1)*knots + j].r.x;
//            currentForm.posCoords[13] = (float) flow->particles[(i+1)*knots + j].r.y;
//            currentForm.posCoords[14] = (float) flow->particles[(i+1)*knots + j].r.z;

//            currentForm.posCoords[0]  = currentForm.posCoords[18] + fabs(currentForm.posCoords[6] - currentForm.posCoords[18]) / 2;
//            currentForm.posCoords[1]  = currentForm.posCoords[19] + fabs(currentForm.posCoords[7] - currentForm.posCoords[19]) / 2;
//            currentForm.posCoords[2]  = currentForm.posCoords[20] + fabs(currentForm.posCoords[8] - currentForm.posCoords[20]) / 2;

//            currentForm.posCoords[30] = currentForm.posCoords[6];
//            currentForm.posCoords[31] = currentForm.posCoords[7];
//            currentForm.posCoords[32] = currentForm.posCoords[8];
//        }
//    }

//    Matrices::mvpMatrix.setToIdentity();
//    Matrices::mvpMatrix = Matrices::projectionMatrix * Matrices::viewMatrix;
//    for (Form s : forms) {
//        s.move();
//        s.draw(/*mvpMatrix*/);
//    }
}