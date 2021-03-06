#include "machine/cloth.h"

#include "gl/matrices.h"
#include "machine/particle.h"
#include "physics/vector.h"
#include "shape/rectangle.h"
#include "util/enums.h"

using namespace std;

// LEGACY //////////////////////////////////////////////////////////////////////

Cloth::~Cloth()
{
}

Cloth::Cloth(
    const Vector& start, const Vector& end,
    int knots, float ks, float d, float kd, int strength
)
    : knots(knots)
{
    color[0] = 1.0f;
    color[1] = 1.0f;
    color[2] = 0.0f;

    Particle::flows.push_back(vector<Particle*>(knots * knots));
    flow = &Particle::flows.back();
    for (int i = 0; i < knots * knots; ++i)
        (*flow)[i] = new Particle(Particle::flows.size() - 1);

    for (int i = 1; i < knots * knots; ++i) // hor-sing
        if (i % knots != 0)
            (*flow)[i]->springifyMutual((*flow)[i-1], ks, d, kd);
    for (int i = 2; i < knots * knots; ++i) // hor-dual
        if (i % knots > 1)
            (*flow)[i]->springifyMutual((*flow)[i-2], ks/2, d*2, kd/2);

    for (int i = knots; i < knots * knots; ++i) // ver-sing
        (*flow)[i]->springifyMutual((*flow)[i - knots], ks, d, kd);
    for (int i = 2 * knots; i < knots * knots; ++i) // ver-dual
        (*flow)[i]->springifyMutual((*flow)[i-(2*knots)], ks/2, d*2, kd/2);

    for (int i = knots + 1; i < knots * knots; ++i) // skw-sing-upright
        if ((i-1)%knots != 0)
            (*flow)[i]->springifyMutual((*flow)[i-knots-1], ks, d, kd);
    for (int i = knots * knots - knots; i > 0; --i) // skw-sing-upleft
        if (i%knots != 0)
            (*flow)[i]->springifyMutual((*flow)[i+knots-1], ks, d, kd);

    Vector rn = start;
    for (int i = 0; i < knots; ++i)
    {
        rn.y += (end.y - start.y) / knots;
        rn.z += (end.z - start.z) / knots;
        rn.x = start.x;
        for (int j = 0; j < knots; ++j)
        {
            rn.x += (end.x - start.x) / knots;
            (*flow)[i*knots + j]->r = new Vector(rn);
            if ((i == 0       && j == 0) || (i == 0       && j == knots-1)
             || (i == knots-1 && j == 0) || (i == knots-1 && j == knots-1))
            {
                (*flow)[i*knots + j]->isStationary = true;
            }
        }
    }

    for (unsigned i = 0; i < (*flow).size(); ++i)
    {
        (*flow)[i]->v = new Vector();
//        p.recolor(color);
    }

    linkView(CLOTH);
}

void Cloth::createView()
{
    vector<float> primaryClothSingularity
            = vector<float>(36 * (knots - 1) * (knots - 1));

    for (unsigned i = 0; i < primaryClothSingularity.size(); i += 6)
    {
        primaryClothSingularity[i+0] = 0; // pos
        primaryClothSingularity[i+1] = 0;
        primaryClothSingularity[i+2] = 0;
        primaryClothSingularity[i+3] = 0; // color
        primaryClothSingularity[i+4] = 1;
        primaryClothSingularity[i+5] = 0;
    }

    currentForm = new Form(
        primaryClothSingularity, 3, GL_TRIANGLE_FAN,
        vector<float>(), 3,
        vector<float>(), 0, 0,
        CLOTH
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

    for (unsigned i = 0; i < (*flow).size(); ++i)
    {
        (*flow)[i]->paint();
    }
}

void Cloth::move()
{
    if (currentForm != nullptr)
    {
        for (int i = 0; i < knots - 1; ++i)
        {
            for (int j = 0; j < knots - 1; ++j)
            {
                int k = (i * (knots - 1) + j) * 36;

                currentForm->posCoords[k+6]  = (*flow)[(i+1)*knots + j + 1]->r->x;
                currentForm->posCoords[k+7]  = (*flow)[(i+1)*knots + j + 1]->r->y;
                currentForm->posCoords[k+8]  = (*flow)[(i+1)*knots + j + 1]->r->z;

                currentForm->posCoords[k+24] = (*flow)[i*knots + j + 1]->r->x;
                currentForm->posCoords[k+25] = (*flow)[i*knots + j + 1]->r->y;
                currentForm->posCoords[k+26] = (*flow)[i*knots + j + 1]->r->z;

                currentForm->posCoords[k+18] = (*flow)[i*knots + j]->r->x;
                currentForm->posCoords[k+19] = (*flow)[i*knots + j]->r->y;
                currentForm->posCoords[k+20] = (*flow)[i*knots + j]->r->z;

                currentForm->posCoords[k+12] = (*flow)[(i+1)*knots + j]->r->x;
                currentForm->posCoords[k+13] = (*flow)[(i+1)*knots + j]->r->y;
                currentForm->posCoords[k+14] = (*flow)[(i+1)*knots + j]->r->z;

                currentForm->posCoords[k+0]  = currentForm->posCoords[k+18]
                                      + fabs(currentForm->posCoords[k+6]
                                      - currentForm->posCoords[k+18]) / 2;
                currentForm->posCoords[k+1]  = currentForm->posCoords[k+19]
                                      + fabs(currentForm->posCoords[k+7]
                                      - currentForm->posCoords[k+19]) / 2;
                currentForm->posCoords[k+2]  = currentForm->posCoords[k+20]
                                      + fabs(currentForm->posCoords[k+8]
                                      - currentForm->posCoords[k+20]) / 2;

                currentForm->posCoords[k+30] = currentForm->posCoords[k+6];
                currentForm->posCoords[k+31] = currentForm->posCoords[k+7];
                currentForm->posCoords[k+32] = currentForm->posCoords[k+8];
            }
        }
        currentForm->move();
    }
}
