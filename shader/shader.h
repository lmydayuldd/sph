#ifndef SHADER_H
#define SHADER_H

#include <QOpenGLShaderProgram>

class Shader {
public:
    static Shader *currentShader;

    const char *vertexShaderSource   = ":/shader/vertex_shader.vsh";
    const char *fragmentShaderSource = ":/shader/fragment_shader.fsh";
    QOpenGLShaderProgram *program;

    ~Shader();
    Shader();

    void bind();
    void release();
    void setUniforms();
};

#endif // SHADER_H
