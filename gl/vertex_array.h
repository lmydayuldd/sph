#ifndef VERTEX_ARRAY_H
#define VERTEX_ARRAY_H

#include <QOpenGLBuffer>
#include <QOpenGLFunctions>
//#include <QOpenGLVertexArrayObject>

#include "gl/form.h" // enum ShapeNames

class VertexArray {
private:
    QOpenGLFunctions* gl;

public:
    static std::vector<VertexArray*> arrays;
    QOpenGLBuffer* vertexBuffer;
    ShapeType type;

    ~VertexArray();
    VertexArray(std::vector<float> vertexData, ShapeType type); // initialize vertex byte buffer for coordinates

    static void printArrays();

    void setVertexAttribPointer(
        int attributeHandle,
        int COORDS_PER_VERTEX,
        int dataStride,
        int dataOffset
    );
};

#endif // VERTEX_ARRAY_H
