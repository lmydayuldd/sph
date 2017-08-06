#include "shader/shader.h"

#include "gl/handles.h"
#include "gl/matrices.h"
#include "util/parser.h"

Shader *Shader::currentShader;

Shader::~Shader()
{
    delete vertexShaderSource;
    delete fragmentShaderSource;
    delete program;
}

Shader::Shader() {
    program = new QOpenGLShaderProgram(0);
    program->create(); // worked without it, created at object creation?
    program->addShaderFromSourceFile(QOpenGLShader::Vertex, vertexShaderSource);
    program->addShaderFromSourceFile(QOpenGLShader::Fragment, fragmentShaderSource);
    program->link();

    Handles::aPos = program->attributeLocation("aPos");
    Handles::aCol = program->attributeLocation("aCol");
    Handles::uMatrix = program->uniformLocation("uMatrix");
}

void Shader::setUniforms() {
    Shader::currentShader->program->setUniformValue(Handles::uMatrix, Matrices::mvpMatrix);
}
