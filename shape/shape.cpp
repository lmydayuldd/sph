#include "shape.h"

Shape::~Shape()
{
    delete form;
}

Shape::Shape()
    : form(nullptr)
{
}
