#include "physics/vector.h"

#include "gl/form.h"
#include "gl/matrices.h"
#include "shape/arrow.h"
#include "util/settings.h"

#include "shape/line.h"
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
    linkView(ARROW);
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

void Vector::collide(Particle* p2) {}
