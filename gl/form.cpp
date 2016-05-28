#include "gl/form.h"

#include <typeinfo>
#include <iostream>

#include <QOpenGLFunctions>

#include "shader/shader.h"
#include "gl/handles.h"
#include "gl/vertex_array.h"
#include "gl/matrices.h"

using namespace std;

vector<Form*> Form::forms;
ShapeType Form::lastBoundFormType = NOTHING;

Form::~Form()
{
    delete gl;
    //delete vertexArray;
}

Form::Form()
    : type(NOTHING)
{
}

Form::Form(
    vector<float> posCoords, int POS_COORDS_PER_VERTEX, int shapeMode,
    vector<float> clrCoords, int CLR_COORDS_PER_VERTEX,
    vector<float> texCoords, int TEX_COORDS_PER_VERTEX, int textureResource,
    ShapeType type
)
    : POS_COORDS_PER_VERTEX(POS_COORDS_PER_VERTEX),
      CLR_COORDS_PER_VERTEX(CLR_COORDS_PER_VERTEX),
      TEX_COORDS_PER_VERTEX(TEX_COORDS_PER_VERTEX),
      ALL_COORDS_PER_VERTEX(POS_COORDS_PER_VERTEX + CLR_COORDS_PER_VERTEX + TEX_COORDS_PER_VERTEX),
      shapeMode(shapeMode),
      textureResource(textureResource),
      vertexArray(new VertexArray(posCoords, type)), // initialize vertex byte buffer for shape coordinates
      posCoords(posCoords),
      clrCoords(clrCoords),
      texCoords(texCoords),
      type(type)
{
    gl = QOpenGLContext::currentContext()->functions();
    vertexCount = posCoords.size() / ALL_COORDS_PER_VERTEX;
    vertexStride = ALL_COORDS_PER_VERTEX * sizeof(float);

    createShaderProgram();

    forms.push_back(this);
}

void Form::printForms()
{
    if (forms.size() > 0) {
#ifdef DESKTOP_BUILD
        string s = to_string(forms.size()) + " Forms:";
#elif ANDROID_BUILD
        string s = "";
#endif
        for (unsigned int i = 0; i < forms.size(); ++i) {
            s += " " + string(Enums::shapeName[forms[i]->type]);
        }
        cout << s << endl;
    }
}

void Form::bindVAO()
{
    lastBoundFormType = type;
    vertexArray->vertexBuffer->bind();
}

void Form::createShaderProgram() {
//        if      (clrCoords != nullptr)                                    shaderProgram = new UniformColorShaderProgram(context);
//        else if (clrCoords == nullptr && COLOR_COORDS_PER_VERTEX > 0)     shaderProgram = new AttributeColorShaderProgram(context);
//        else if (texCoords != nullptr || TEXTURE_COORDS_PER_VERTEX > 0) shaderProgram = new TextureShaderProgram(context);
//        else                                                             shaderProgram = nullptr;
}

void Form::bindUniforms()
{
//    gl->glUniformMatrix4fv(
//        Handles::uMatrix,
//        GL_FALSE,
//        1,
//        Matrices::mvpMatrix.constData()
//    );
    Shader::currentShader->setUniforms();
}

void Form::bindAttributes()
{
//  if      (shaderProgram instanceof AttributeColorShaderProgram) {
    vertexArray->setVertexAttribPointer(
        Handles::aPos,
        POS_COORDS_PER_VERTEX,
        vertexStride,
        0
    );
    vertexArray->setVertexAttribPointer(
        Handles::aCol,
        CLR_COORDS_PER_VERTEX,
        vertexStride,
        POS_COORDS_PER_VERTEX
    );
//  }
}

void Form::disableVertexAttribArrays()
{
//  if      (shaderProgram instanceof AttributeColorShaderProgram) {
    gl->glDisableVertexAttribArray(Handles::aPos);
    gl->glDisableVertexAttribArray(Handles::aCol);
//  }
}

void Form::draw()
{
    //vertexArray->vertexBuffer->bind();
    bindUniforms(); // glEnableVertexAttribArray() calls are in here
    bindAttributes();
        gl->glDrawArrays(shapeMode, 0, vertexCount);
        //gl->glDrawElements(shapeMode, posCoords.length, GL_UNSIGNED_SHORT, vertexBuffer);
    disableVertexAttribArrays();
}

void Form::move()
{
    vertexArray = new VertexArray(posCoords, type);
}

void Form::recolor(float color[])
{
//    if      (shaderProgram instanceof AttributeColorShaderProgram) {
        for (int i = POS_COORDS_PER_VERTEX; i < (int) posCoords.size(); i += ALL_COORDS_PER_VERTEX)
            for (int j = 0; j < CLR_COORDS_PER_VERTEX; ++j)
                posCoords[i + j] = color[j];
        move();
//    }
//    else if (shaderProgram instanceof UniformColorShaderProgram) {
//        System.arraycopy(color, 0, clrCoords, 0, COLOR_COORDS_PER_VERTEX);
//    }
}
