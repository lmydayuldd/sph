#ifndef VERTEX_ARRAY_H
#define VERTEX_ARRAY_H

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
//#include <QOpenGLVertexArrayObject>

#include "util/enums.h"

class VertexArray {
private:
    QOpenGLFunctions* gl;

public:
    static std::vector<VertexArray*> arrays;
    QOpenGLBuffer* vertexBuffer;
    ShapeType type;

    ~VertexArray();
    // initialize vertex byte buffer for coordinates
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
