#ifndef FORM_H
#define FORM_H

#include <QOpenGLFunctions>

class VertexArray;

enum ShapeNames {
    NOTHING,
    DOT,
    LINE,
    ARROW,
    TRIANGLE,
    RECTANGLE,
    SPHERE,
    LANDSCHAFT
};

class Form {
private:
    int POS_COORDS_PER_VERTEX = 0;
    int CLR_COORDS_PER_VERTEX = 0;
    int TEX_COORDS_PER_VERTEX = 0;
    int ALL_COORDS_PER_VERTEX = 0;
    int vertexCount     = 0;
    int vertexStride    = 0;
    int shapeMode       = 0;
    int textureResource = 0;
    QOpenGLFunctions* gl;

    void bindUniforms();
    void bindAttributes();
    void disableVertexAttribArrays();

public:
    static std::vector<Form> forms;
    static ShapeNames lastBoundFormName;
    VertexArray* vertexArray;
    std::vector<float> posCoords;
    std::vector<float> clrCoords;
    std::vector<float> texCoords;
    ShapeNames name;

    Form();
    Form(
        std::vector<float> posCoords, int POS_COORDS_PER_VERTEX, int shapeMode,
        std::vector<float> clrCoords, int COL_COORDS_PER_VERTEX,
        std::vector<float> texCoords, int TEX_COORDS_PER_VERTEX, int textureResource,
        ShapeNames name
    );
    ~Form();

    static void printForms();
    void bindVAO();
    void createShaderProgram();
    void recolor(float color[]);
    void move();
    void draw();
};

#endif // FORM_H
