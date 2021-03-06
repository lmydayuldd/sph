#include "physics/vector.h"

#include "gl/form.h"
#include "gl/matrices.h"
#include "shape/arrow.h"
#include "shape/line.h"
#include "util/operations.h"
#include "util/settings.h"

#include <iostream>

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
    this->currentForm = arrow.form;

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
Vector Vector::operator+(const Vector &v) const {
    return Vector(this->x+v.x, this->y+v.y, this->z+v.z);
}
Vector Vector::operator-(const Vector &v) const {
    return Vector(this->x-v.x, this->y-v.y, this->z-v.z);
}
Vector Vector::operator*(const Vector &v) const { // cross
    //return Vector(this->x*v.x, this->y*v.y, this->z*v.z);
    return Vector(
        this->y*v.z - this->z*v.y,
        this->z*v.x - this->x*v.z,
        this->x*v.y - this->y*v.x
    );
}
//double Vector::operator/(const Vector& v) const { // dot
//    return dot(v);
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
Vector& Vector::operator+=(const Vector &v) {
    this->x += v.x;
    this->y += v.y;
    this->z += v.z;
    return *this;
}
Vector& Vector::operator-=(const Vector &v) {
    this->x -= v.x;
    this->y -= v.y;
    this->z -= v.z;
    return *this;
}
Vector& Vector::operator*=(const Vector &v) {
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

double Vector::cosxy(const Vector &v) const {
    return dot(v) / (this->norm() * v.norm());
}
double Vector::distance(const Vector &v) const {
    return (*this - v).norm();
}
double Vector::dot(const Vector &v) const {
    return this->x*v.x + this->y*v.y + this->z*v.z;
}

double Vector::norm() const {
    return sqrt(this->dot(*this));
}
Vector Vector::normal() const {
    return *this / this->norm();
}

void Vector::limit(double max) {
    if (fabs(x) > max) x = Op::sgn(x) * max;
    if (fabs(y) > max) y = Op::sgn(y) * max;
    if (fabs(z) > max) z = Op::sgn(z) * max;
}

void Vector::cut(double max) {
   *this = this->normal() * max;
}

void Vector::zero() {
    x = 0;
    y = 0;
    z = 0;
}

void Vector::setModelMatrix()
{
    double us = norm() * 0.2 * Settings::VECTOR_LEN_MULT;
    Matrices::modelMatrix.scale(QVector3D(us, us, us));
}

void Vector::paint()
{
    if (Settings::PAINT_VECTORS) {
        setModelMatrix();
        Machine::paint();
    }
}
