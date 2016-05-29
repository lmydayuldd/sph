#ifndef ENUMS_H
#define ENUMS_H

enum ShapeType : unsigned char {
    NOTHING,
    DOT,
    LINE,
    ARROW,
    TRIANGLE,
    RECTANGLE,
    SPHERE,
    LANDSCHAFT,
    CLOTH
};

class Enums
{
public:
    static const char* shapeName[];
};

#endif // ENUMS_H
