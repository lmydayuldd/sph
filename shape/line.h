#ifndef LINE_H
#define LINE_H

#include "shape/shape.h"

class Line : public Shape {
public:
    Line(std::vector<float> ends, std::vector<float> color);

    void makeForm();
    void move(float x1, float y1, float x2, float y2);
    void recolor(float color[]);
};

#endif // LINE_H
