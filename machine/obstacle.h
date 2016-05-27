#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <vector>

class Particle;

class Obstacle
{
public:
    static std::vector<Obstacle> obstacles;

    Obstacle();

    virtual void collide(Particle* p2) = 0;
};

#endif // OBSTACLE_H
