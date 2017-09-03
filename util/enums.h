#ifndef ENUMS_H
#define ENUMS_H

enum ShapeType : unsigned char {
    NOTHING,
    DOT,
    LINE,
    ARROW,
    TRIANGLE,
    RECTANGLE,
    SPHERE, SPHERE_BOUNDARY, SPHERE_OVERPRESSURED,
    LANDSCHAFT,
    CLOTH
};

enum MapSetup : unsigned char {
    RANDOM_NON_MAP, DAM_BREAK_NON_MAP, DAM_BREAK_3D_NON_MAP,
    DAM_BREAK, DROPLET, VESSELS, DAM_FALL
};

enum Key : unsigned char {
    W, S, A, D, DUCK,
    CTRL, LMB, RMB,
    SPACE, BACKSPACE, ESCAPE,
    PARALLEL,
    RENDER, CONTROL
};

enum DragMode : unsigned char {
    ONE_DRAG, LOCAL_DRAG, FORCE_DRAG
};

class Enums
{
public:
    static const char* shapeName[];
};

#endif // ENUMS_H
