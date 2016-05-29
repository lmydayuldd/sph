#ifndef LINE_H
#define LINE_H

#include "shape/shape.h"

class Line : public Shape {
public:
    Line(std::vector<float> ends, std::vector<float> color);

    void move(float x1, float y1, float x2, float y2);
    void recolor(float color[]);
    virtual void makeForm() override;
};

#endif // LINE_H
