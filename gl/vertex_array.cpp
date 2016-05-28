#include "gl/vertex_array.h"

#include <iostream>

using namespace std;

vector<VertexArray*> VertexArray::arrays;

VertexArray::~VertexArray() {
    delete gl;
    if (find(arrays.begin(), arrays.end(), this) == arrays.end()) {
        delete vertexBuffer;
    }
}

// initialize vertex byte buffer for coordinates
VertexArray::VertexArray(vector<float> vertexData, ShapeType type)
    : vertexBuffer(new QOpenGLBuffer(QOpenGLBuffer::VertexBuffer)),
      type(type)
{
    //cout << QOpenGLContext::currentContext() << " ";
    vertexBuffer->create();
    vertexBuffer->setUsagePattern(QOpenGLBuffer::StaticDraw); // DynamicDraw, StreamDraw, ...?
    vertexBuffer->bind();
    vertexBuffer->allocate(vertexData.data(), vertexData.size() * sizeof(float));
    vertexBuffer->write(0, vertexData.data(), vertexData.size() * sizeof(float));

    arrays.push_back(this);
}

void VertexArray::printArrays()
{
    if (arrays.size() > 0) {
#ifdef DESKTOP_BUILD
        string s = to_string(arrays.size()) + " VAOs:";
#elif ANDROID_BUILD
        string s = "";
#endif
        for (unsigned int i = 0; i < arrays.size(); ++i) {
            s += " " + string(Enums::shapeName[arrays[i]->type]);
        }
        cout << s << endl;
    }
}

void VertexArray::setVertexAttribPointer(
    int attributeHandle,
    int COORDS_PER_VERTEX,
    int dataStride,
    int dataOffset
) {
    QOpenGLFunctions* gl = QOpenGLContext::currentContext()->functions();
    gl->glVertexAttribPointer(
        attributeHandle,
        COORDS_PER_VERTEX,
        GL_FLOAT,
        GL_FALSE,
        dataStride,
        (GLvoid*)(sizeof(float) * dataOffset)
    );
    gl->glEnableVertexAttribArray(attributeHandle);
}
