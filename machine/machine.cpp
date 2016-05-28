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

void Machine::linkView(ShapeTypes formShapeType)
{
    if (form == nullptr) {
        if (Form::forms.size() == 0) {
            createView();
        }
        else {
            for (unsigned int i = 0; i < Form::forms.size(); ++i) {
                if (Form::forms[i]->type == formShapeType) {
                    form = Form::forms[i];
                    break; ///////////////////////////////////////////////////
                }
                else if (i == Form::forms.size() - 1) {
                    createView();
                }
            }
        }
    }
}

void Machine::paint()
{
    if (form != nullptr) {
        if (Form::lastBoundFormType != form->type) {
           form->bindVAO();
        }
        Matrices::modelViewMatrix.setToIdentity();
        Matrices::mvpMatrix.setToIdentity();
        Matrices::modelViewMatrix = Matrices::viewMatrix       * Matrices::modelMatrix;
        Matrices::mvpMatrix       = Matrices::projectionMatrix * Matrices::modelViewMatrix;

        form->draw();
    }
}

void Machine::createView() {}
void Machine::setModelMatrix() {}

void Machine::recolor(float color[])
{
    form->recolor(color);
}
