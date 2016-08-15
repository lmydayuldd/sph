#include "physics/vector.h"

#include <iostream>

#include "gl/form.h"
#include "gl/matrices.h"
#include "shape/arrow.h"
#include "shape/line.h"
#include "util/settings.h"

using namespace std;

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
#ifndef TEST_BUILD
    linkView(ARROW);
#endif
}

Vector::Vector(const Vector& v)
{
    *this = Vector(v.x, v.y, v.z);
}

void Vector::createView()
{
    Arrow arrow;
    this->form = arrow.form;

//    vector<float> vertices;
//    vector<float> color = {1, 1, 0};
//    vector<float> a = {0, 0,  0.0, 1.0};
//    vector<float> b = {0, 1, -0.1, 0.9};
//    vector<float> c = {0, 1,  0.1, 0.9};
//    Line l1 = Line(a, color);
//    Line l2 = Line(b, color);
//    Line l3 = Line(c, color);
//    vertices.insert(vertices.end(), l1.vertices.begin(), l1.vertices.end());
//    vertices.insert(vertices.end(), l2.vertices.begin(), l2.vertices.end());
//    vertices.insert(vertices.end(), l3.vertices.begin(), l3.vertices.end());
//    int posCoordsPerVertex = 2;
//    int clrCoordsPerVertex = 3;
//    form = new Form(
//        vertices, posCoordsPerVertex, GL_LINES,
//        vector<float>(), clrCoordsPerVertex,
//        vector<float>(), 0, 0,
//        ARROW
//    );
}

void Vector::setApplicationPoint(Vector application)
{
    source = &application;
}

bool Vector::operator==(const Vector& v) const {
    bool ret = true;
    if (this->x != v.x
     || this->y != v.y
     || this->z != v.z)
//     || this->source != v.source
//     || this->source->x != v.source->x
//     || this->source->y != v.source->y
//     || this->source->z != v.source->z)
    {
        ret = false;
    }
    return ret;
}
Vector Vector::operator-() const {
    return Vector(- this->x, - this->y, - this->z);
}
Vector Vector::operator+(const Vector& v) const {
    return Vector(this->x+v.x, this->y+v.y, this->z+v.z);
}
Vector Vector::operator-(const Vector& v) const {
    return Vector(this->x-v.x, this->y-v.y, this->z-v.z);
}
Vector Vector::operator*(const Vector& v) const { // cross
    //return Vector(this->x*v.x, this->y*v.y, this->z*v.z);
    return Vector(
        this->y*v.z - this->z*v.y,
        this->z*v.x - this->x*v.z,
        this->x*v.y - this->y*v.x
    );
}
double Vector::operator/(const Vector& v) const { // dot
    return dotProduct(v);
}
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
Vector& Vector::operator+=(const Vector& v) {
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    return *this;
}
Vector& Vector::operator-=(const Vector& v) {
    this->x -= v.x;
    this->y -= v.y;
    this->z -= v.z;
    return *this;
}
Vector& Vector::operator*=(const Vector& v) {
    *this = Vector(
        this->y*v.z - this->z*v.y,
        this->z*v.x - this->x*v.z,
        this->x*v.y - this->y*v.x
    );
    return *this;
}
Vector& Vector::operator+=(double v) {
    this->x += v;
    this->y += v;
    this->z += v;
    return *this;
}
Vector& Vector::operator-=(double v) {
    this->x -= v;
    this->y -= v;
    this->z -= v;
    return *this;
}
Vector& Vector::operator*=(double v) {
    this->x *= v;
    this->y *= v;
    this->z *= v;
    return *this;
}
Vector& Vector::operator/=(double v) {
    this->x /= v;
    this->y /= v;
    this->z /= v;
    return *this;
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

double Vector::norm() const {
    return sqrt(this->dotProduct(*this));
}
Vector Vector::normal() const {
    return *this / this->norm();
}

void Vector::limit(double max) {
    auto sgn = [](double x) -> double { return x < 0 ? -1 : 1; };
    if (fabs(x) > max) x = sgn(x) * max;
    if (fabs(y) > max) y = sgn(y) * max;
    if (fabs(z) > max) z = sgn(z) * max;
}

void Vector::setModelMatrix()
{
    double us = norm() * 0.2 * Settings::VECTOR_LENGTH_MULTIPLIER;
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

void Vector::collide(Particle* p2) {}
