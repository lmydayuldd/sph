#include "machine/machine.h"

#include "gl/form.h"
#include "gl/matrices.h"

using namespace std;

vector<Machine*> Machine::machines;

Machine::Machine()
    : form(nullptr)
{
}

Machine::~Machine()
{
    //delete form;
}

void Machine::linkView(ShapeType formShapeType)
{
    if (form == nullptr)
    {
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
                    form = Form::forms[i];
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
    }
}

void Machine::paint()
{
    if (form != nullptr)
    {
        if (Form::lastBoundFormType != form->type)
        {
           form->bindVAO();
        }
        Matrices::modelViewMatrix.setToIdentity();
        Matrices::mvpMatrix.setToIdentity();
        Matrices::modelViewMatrix = Matrices::viewMatrix
                                  * Matrices::modelMatrix;
        Matrices::mvpMatrix       = Matrices::projectionMatrix
                                  * Matrices::modelViewMatrix;

        form->draw();
    }
}

void Machine::createView() {}
void Machine::setModelMatrix() {}

void Machine::recolor(float color[])
{
    form->recolor(color);
}
