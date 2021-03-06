#include "machine/machine.h"

#include "gl/form.h"
#include "gl/matrices.h"

using namespace std;

vector<Machine*> Machine::machines;

Machine::Machine()
    : currentForm(nullptr)
{
}

Machine::~Machine()
{
}

void Machine::linkView(ShapeType formShapeType)
{
    //if (form == nullptr)
    //{
        if (Form::forms.size() == 0)
        {
            createView();
        }
        else
        {
            for (unsigned i = 0; i < Form::forms.size(); ++i)
            {
                if (Form::forms[i]->type == formShapeType)
                {
                    currentForm = Form::forms[i];
                    break; ///////////////////////////////////////////////////
                }
                else if (i == Form::forms.size() - 1)
                {
                    createView();
                    break; ///////////////////////////////////////////////////
                    // without break it shouldn't fail as well,
                    // because at createView() Form::forms is appended
                    // and at next loop run we'll hit first break
                }
            }
        }
    //}
}

void Machine::paint()
{
    if (currentForm != nullptr)
    {
        if (Form::lastBoundFormType != currentForm->type)
        {
           currentForm->bindVAO();
        }
        Matrices::modelViewMatrix.setToIdentity();
        Matrices::mvpMatrix.setToIdentity();
        Matrices::modelViewMatrix = Matrices::viewMatrix
                                  * Matrices::modelMatrix;
        Matrices::mvpMatrix       = Matrices::projectionMatrix
                                  * Matrices::modelViewMatrix;

        currentForm->draw();
    }
}

void Machine::recolor(float color[])
{
    currentForm->recolor(color);
}
