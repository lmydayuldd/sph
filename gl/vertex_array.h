#ifndef VERTEX_ARRAY_H
#define VERTEX_ARRAY_H

#include "util/enums.h"

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
//#include <QOpenGLVertexArrayObject>

class VertexArray {
private:
    QOpenGLFunctions *gl;

public:
    static std::vector<VertexArray*> arrays;
    QOpenGLBuffer *vertexBuffer;
    ShapeType type;

    ~VertexArray();
    VertexArray(std::vector<float> vertexData, ShapeType type);

    static void printArrays();

    void setVertexAttribPointer(
        int attributeHandle,
        int COORDS_PER_VERTEX,
        int dataStride,
        int dataOffset
    );
};

#endif // VERTEX_ARRAY_H
