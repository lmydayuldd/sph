#include "machine/cloth.h"

#include "gl/matrices.h"
#include "machine/particle.h"
#include "physics/vector.h"
#include "shape/rectangle.h"
#include "util/enums.h"

using namespace std;

Cloth::~Cloth()
{
}

Cloth::Cloth(
    const Vector& start, const Vector& end,
    int knots, float ks, float d, float kd, int strength
)
    : flow(vector<Particle>(knots * knots)),
      knots(knots)
{
    color[0] = 1.0f;
    color[1] = 1.0f;
    color[2] = 0.0f;

    for (int i = 1; i < knots * knots; ++i) // hor-sing
        if (i % knots != 0)
            flow[i].springifyMutual(&flow[i - 1], ks, d, kd);
    for (int i = 2; i < knots * knots; ++i) // hor-dual
        if (i % knots > 1)
            flow[i].springifyMutual(&flow[i-2], ks/2, d*2, kd/2);

    for (int i = knots; i < knots * knots; ++i) // ver-sing
        flow[i].springifyMutual(&flow[i - knots], ks, d, kd);
    for (int i = 2 * knots; i < knots * knots; ++i) // ver-dual
        flow[i].springifyMutual(&flow[i-(2*knots)], ks/2, d*2, kd/2);

    for (int i = knots + 1; i < knots * knots; ++i) // skw-sing-upright
        if ((i-1)%knots != 0)
            flow[i].springifyMutual(&flow[i-knots-1], ks, d, kd);
    for (int i = knots * knots - knots; i > 0; --i) // skw-sing-upleft
        if (i%knots != 0)
            flow[i].springifyMutual(&flow[i+knots-1], ks, d, kd);

    double ny = start.y;
    double nz = start.z;
    for (int i = 0; i < knots; ++i) {
        ny += (end.y - start.y) / knots;
        nz += (end.z - start.z) / knots;
        double nx = start.x;
        for (int j = 0; j < knots; ++j) {
            nx += (end.x - start.x) / knots;
            flow[i*knots + j].r->x = nx;
            flow[i*knots + j].r->y = ny;
            flow[i*knots + j].r->z = nz;
            if ((i == 0 && j == 0) || (i == 0 && j == knots-1) || (i == knots-1 && j == 0) || (i == knots-1 && j == knots-1)) {
                flow[i*knots + j].stationary = true;
            }
        }
    }

    for (Particle p : flow) p.v = new Vector();
    //for (Particle p : flow) p.recolor(color);

    linkView(LANDSCHAFT);
}

void Cloth::createView()
{
    vector<float> primaryClothSingularity
            = vector<float>(36 * (knots - 1) * (knots - 1));

    for (unsigned int i = 0; i < primaryClothSingularity.size(); i += 6) {
        primaryClothSingularity[i+0] = 0; // pos
        primaryClothSingularity[i+1] = 0;
        primaryClothSingularity[i+2] = 0;
        primaryClothSingularity[i+3] = 0; // color
        primaryClothSingularity[i+4] = 1;
        primaryClothSingularity[i+5] = 0;
    }

    form = new Form(
        primaryClothSingularity, 3, GL_TRIANGLE_FAN,
        vector<float>(), 3,
        vector<float>(), 0, 0,
        LANDSCHAFT
    );
}

void Cloth::setModelMatrix()
{
    Matrices::modelMatrix.setToIdentity();
}

void Cloth::paint()
{
    move();

    setModelMatrix();

    Machine::paint();
}

void Cloth::move()
{
    if (form != nullptr) {
        for (int i = 0; i < knots - 1; ++i) {
            for (int j = 0; j < knots - 1; ++j) {
                int k = i * (knots - 1) + j;

                form->posCoords[k+6]  = flow[(i+1)*knots + j + 1].r->x;
                form->posCoords[k+7]  = flow[(i+1)*knots + j + 1].r->y;
                form->posCoords[k+8]  = flow[(i+1)*knots + j + 1].r->z;

                form->posCoords[k+24] = flow[i*knots + j + 1].r->x;
                form->posCoords[k+25] = flow[i*knots + j + 1].r->y;
                form->posCoords[k+26] = flow[i*knots + j + 1].r->z;

                form->posCoords[k+18] = flow[i*knots + j].r->x;
                form->posCoords[k+19] = flow[i*knots + j].r->y;
                form->posCoords[k+20] = flow[i*knots + j].r->z;

                form->posCoords[k+12] = flow[(i+1)*knots + j].r->x;
                form->posCoords[k+13] = flow[(i+1)*knots + j].r->y;
                form->posCoords[k+14] = flow[(i+1)*knots + j].r->z;

                form->posCoords[k+0]  = form->posCoords[k+18] + fabs(form->posCoords[k+6] - form->posCoords[k+18]) / 2;
                form->posCoords[k+1]  = form->posCoords[k+19] + fabs(form->posCoords[k+7] - form->posCoords[k+19]) / 2;
                form->posCoords[k+2]  = form->posCoords[k+20] + fabs(form->posCoords[k+8] - form->posCoords[k+20]) / 2;

                form->posCoords[k+30] = form->posCoords[k+6];
                form->posCoords[k+31] = form->posCoords[k+7];
                form->posCoords[k+32] = form->posCoords[k+8];
            }
        }
        form->move();
    }
}

void Cloth::collide(Particle* p2) {}
