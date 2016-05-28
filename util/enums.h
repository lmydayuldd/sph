#ifndef ENUMS_H
#define ENUMS_H

enum ShapeTypes : unsigned char {
    NOTHING,
    DOT,
    LINE,
    ARROW,
    TRIANGLE,
    RECTANGLE,
    SPHERE,
    LANDSCHAFT
};

class Enums
{
public:
    static const char* shapeNames[];
};

#endif // ENUMS_H
