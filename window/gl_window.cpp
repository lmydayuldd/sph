#include <QPainter>
#include <QCoreApplication>

#include "window/gl_window.h"
#include "gl/matrices.h"
#include "ui_main_window.h"

GLWindow::~GLWindow()
{
    delete glContext;
    delete paintDevice;
}

GLWindow::GLWindow(QWindow *parent) :
    //QMainWindow(parent),
    QWindow(parent),
    //ui(new Ui::MainWindow),
    m_update_pending(false),
    m_animating(false),
    glContext(0),
    paintDevice(0)
{
    //ui->setupUi(this);
    setSurfaceType(QWindow::OpenGLSurface);
}

void GLWindow::render(QPainter *painter)
{
    Q_UNUSED(painter);
}

void GLWindow::initialize()
{
}

void GLWindow::render()
{
    if (! paintDevice)
        paintDevice = new QOpenGLPaintDevice;

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);

    paintDevice->setSize(size());

    QPainter painter(paintDevice);
    render(&painter);
}
void GLWindow::renderLater()
{
    if (! m_update_pending) {
        m_update_pending = true;
        QCoreApplication::postEvent( this, new QEvent(QEvent::UpdateRequest) );
    }
}

bool GLWindow::event( QEvent *event )
{
    switch (event->type()) {
    case QEvent::UpdateRequest:
        m_update_pending = false;
        renderNow();
        return true;
    default:
        return QWindow::event(event);
    }
}

void GLWindow::exposeEvent( QExposeEvent *event )
{
    Q_UNUSED(event);

    if (isExposed())
        renderNow();
}

void GLWindow::renderNow()
{
    if (! isExposed())
        return;

    bool needsInitialize = false;

    if (! glContext) {
        glContext = new QOpenGLContext(this);
        glContext->setFormat(requestedFormat());
        glContext->create();

        needsInitialize = true;
    }

    glContext->makeCurrent(this);

    if (needsInitialize) {
        initializeOpenGLFunctions();
        initialize();
    }

    render();

    glContext->swapBuffers(this);

    if (m_animating)
        renderLater();
}

void GLWindow::setAnimating(bool animating)
{
    m_animating = animating;

    if (animating)
        renderLater();
}
