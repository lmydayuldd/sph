#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

class Constants
{
public:
    static constexpr double degToRad = 2 * M_PI / 360.0;
    static constexpr double radToDeg = 360.0 / (2 * M_PI);
    Constants();
};

#endif // CONSTANTS_H
