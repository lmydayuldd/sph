#ifndef CONSTANTS_H
#define CONSTANTS_H

//#define _USE_MATH_DEFINES
#include <cmath>
#ifdef COMPILER_MSVC
static const double M_PI = 3.14159265358979323846264338327950288;
#endif

static constexpr double degToRad = 2 * M_PI / 360;
static constexpr double radToDeg = 360.0 / (2 * M_PI);

static /*constexpr */double G_const = 6.673 * pow(10, -11);
static /*constexpr */double sigma_0 = 8.8541878176 * pow(10, -12); // electric permittivity of free space (vacuum) [F/m] [farads/m]
static /*constexpr */double Coulomb_const = 8.987 * pow(10, 9); //1 / (4 * PI * sigma_0); // Coulomb force constant [ N * m^2/C^2 ]
//static constexpr double G_earth = 9.81;

#endif // CONSTANTS_H
