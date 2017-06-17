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
    if (find(forms.begin(), forms.end(), this) == forms.end())
    {
        delete vertexArray;
    }
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
      vertexArray(new VertexArray(posCoords, type)),
      posCoords(posCoords),
      clrCoords(clrCoords),
      texCoords(texCoords),
      type(type)
{
    gl = QOpenGLContext::currentContext()->functions();
    vertexCount = posCoords.size() / ALL_COORDS_PER_VERTEX;
    vertexStride = ALL_COORDS_PER_VERTEX * sizeof(float);

    forms.push_back(this);
}

void Form::printForms()
{
    if (forms.size() > 0)
    {
#ifdef DESKTOP_BUILD
        string s = to_string(forms.size()) + " Forms:";
#elif ANDROID_BUILD
        string s = "";
#endif
        for (unsigned int i = 0; i < forms.size(); ++i)
        {
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
    bindUniforms();
    bindAttributes(); // glEnableVertexAttribArray() calls are in here
        gl->glDrawArrays(shapeMode, 0, vertexCount);
//        gl->glDrawElements(
//                shapeMode, posCoords.length, GL_UNSIGNED_SHORT, vertexBuffer);
    disableVertexAttribArrays();
}

void Form::move()
{
    if (vertexArray == nullptr)
    {
        vertexArray = new VertexArray(posCoords, type);
    }
    else
    {
        vertexArray->vertexBuffer->bind();
        vertexArray->vertexBuffer->write(0, posCoords.data(), posCoords.size() * sizeof(float));
   }
}

void Form::recolor(float color[])
{
//    if      (shaderProgram instanceof AttributeColorShaderProgram) {
        float step = M_PI / 23;
        for (int i = POS_COORDS_PER_VERTEX;
             i < (int) posCoords.size();
             i += ALL_COORDS_PER_VERTEX)
        {
            color[1] += step / M_PI;
            if      (color[1] >= 1.0f) color[1] -= fmod(color[1], 1);
            else if (color[1] <= 0.0f) color[1] += fmod(fabs(color[1]), 1);
            for (int j = 0; j < CLR_COORDS_PER_VERTEX; ++j)
            {
                posCoords[i + j] = color[j];
            }
        }
        move();
//    }
//    else if (shaderProgram instanceof UniformColorShaderProgram) {
//        System.arraycopy(color, 0, clrCoords, 0, COLOR_COORDS_PER_VERTEX);
//    }
}
