#include "shape/arrow.h"
#include "gl/form.h"
#include "gl/matrices.h"
#include "util/settings.h"
#include "physics/vector.h"

Vector::~Vector()
{
    delete source;
}

Vector::Vector()
{
    *this = Vector(0, 0, 0);
}

Vector::Vector(double x, double y, double z)
    : x(x), y(y), z(z),
      source(nullptr)
{
    linkView(ARROW);
}

void Vector::createView()
{
    Arrow arrow;
    this->form = arrow.form;
}

void Vector::setApplicationPoint(Vector application)
{
    source = &application;
}

Vector Vector::operator+(const Vector& v) const {
    return Vector(this->x+v.x, this->y+v.y, this->z+v.z);
}
Vector Vector::operator-(const Vector& v) const {
    return Vector(this->x-v.x, this->y-v.y, this->z-v.z);
}
Vector Vector::operator*(const Vector& v) const { // crossProduct
    //return Vector(this->x*v.x, this->y*v.y, this->z*v.z);
    return Vector(
        this->y*v.z - this->z*v.y,
        this->x*v.z - this->z*v.x,
        this->x*v.y - this->y*v.x
    );
}
//Vector Vector::operator/(const Vector& v) const {
//    return Vector(this->x/v.x, this->y/v.y, this->z/v.z);
//}
Vector Vector::operator+(double v) const {
    return Vector(this->x+v, this->y+v, this->z+v);
}
Vector Vector::operator-(double v) const {
    return Vector(this->x-v, this->y-v, this->z-v);
}
Vector Vector::operator*(double v) const {
    return Vector(this->x*v, this->y*v, this->z*v);
}
Vector Vector::operator/(double v) const {
    return Vector(this->x/v, this->y/v, this->z/v);
}
Vector Vector::operator-() const {
    return Vector(- this->x, - this->y, - this->z);
}

double Vector::cosxy(const Vector& v) const {
    return dotProduct(v) / (this->norm() * v.norm());
}
double Vector::distance(const Vector& v) const {
    return (*this - v).norm();
}
double Vector::dotProduct(const Vector& v) const {
    return this->x*v.x + this->y*v.y + this->z*v.z;
}
//Vector Vector::crosssProduct(const Vector& v) const {
//}

double Vector::norm() const {
    return sqrt(dotProduct(*this));
}
Vector Vector::normal() const {
    return *this / this->norm();
}

void Vector::setModelMatrix()
{
    //Matrices::modelMatrix.setToIdentity();
    double us = norm() * 0.2;
    Matrices::modelMatrix.scale(QVector3D(us, us, us));
    //Matrices::modelMatrix.translate(x_0, y_0, z_0);
}

void Vector::paint()
{
    if (Settings::PAINT_VECTORS) {
        setModelMatrix();

        Machine::paint();
    }
}

void Vector::collide(Particle* p2) { p2 = p2; }
