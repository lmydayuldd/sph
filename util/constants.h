#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <cmath>

static constexpr double degToRad = 2 * M_PI / 360.0;
static constexpr double radToDeg = 360.0 / (2 * M_PI);

static constexpr double G_const = 6.673 * pow(10, -11);
//static constexpr double G_const = 6.673 * pow(10, -1);
static constexpr double sigma_0 = 8.8541878176 * pow(10, -12); // electric permittivity of free space (vacuum) [F/m] [farads/m]
static constexpr double Coulomb_const = 8.987 * pow(10, 9); //1 / (4 * PI * sigma_0); // Coulomb force constant [ N * m^2/C^2 ]
//static constexpr double coefficient_of_friction = Settings.FORCES_FRICTION;
static constexpr double coefficient_of_friction = 0.5;
static constexpr double G_earth = 9.81;
//static constexpr double G_earth = 0;

#endif // CONSTANTS_H
