#ifndef ARROW_H
#define ARROW_H

#include "shape/shape.h"

class Arrow : public Shape {
public:
    Arrow();

    virtual void makeForm() override;
};

#endif // ARROW_H
