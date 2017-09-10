#ifndef ENUMS_H
#define ENUMS_H

class Enums
{
public:
    static const char *shapeName[];
};

// update shapeName[] in .cpp file according to changes below!
enum ShapeType : unsigned char {
    NOTHING,
    DOT,
    LINE,
    ARROW,
    TRIANGLE,
    RECTANGLE,
    SPHERE_YELLOW, SPHERE_BLUE, SPHERE_RED, SPHERE_GREEN,
    LANDSCHAFT,
    CLOTH
};

enum IntegrationScheme : unsigned char {
    EULER,
    REVERSE_EULER,
    LEAPFROG,
    MIDPOINT, // RK2
    RK4 // Range-Kutta
};

enum MapSetup : unsigned char {
    RANDOM_NON_MAP,
    DAM_BREAK_NON_MAP,
    DAM_BREAK_3D_NON_MAP,
    DAM_BREAK,
    DROPLET,
    VESSELS,
    DAM_FALL
};

enum Key : unsigned char {
    W, S, A, D, DUCK,
    CTRL, LMB, RMB,
    SPACE, BACKSPACE, ESCAPE,
    PARALLEL, RENDER, CONTROL, ROTATE
};

enum DragMode : unsigned char {
    ONE_DRAG, LOCAL_DRAG, FORCE_DRAG
};

enum ColorBy : unsigned char {
    LAYERS,
    BOUNDARIES,
    DENSITY,
    VELOCITY,
    PRESSURE,
    VISCOSITY,
    TENSION
};

enum Kernel : unsigned char {
   GAUSS, MONAGHAN, CUBIC, SUPER_GAUSS, QUINTIC_LIU
};

#endif // ENUMS_H
