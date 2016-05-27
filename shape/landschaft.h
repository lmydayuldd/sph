#ifndef LANDSCHAFT_H
#define LANDSCHAFT_H

#include "shape/shape.h"

class Landschaft : public Shape {
private:
    int width, height;
    std::vector<std::vector<float>> heightMap;

public:
    Landschaft(std::vector<std::vector<float>> heightMap);

    void sculpt(std::vector<std::vector<float>> heightMap);
};

#endif // LANDSCHAFT_H
