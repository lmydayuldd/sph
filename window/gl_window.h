#ifndef GL_WINDOW_H
#define GL_WINDOW_H

#include <QOpenGLFunctions>
#include <QOpenGLPaintDevice>
#include <QWindow>
#include <QMainWindow>

class GLWindow
        : public QWindow,
          protected QOpenGLFunctions
{
    Q_OBJECT

public:
    explicit GLWindow(QWindow* parent = 0);
    ~GLWindow();

    virtual void render(QPainter* painter);
    virtual void render();
    virtual void initialize();

    void setAnimating(bool animating);

public slots:
    void renderLater();
    void renderNow();

protected:
    bool event(QEvent* event) Q_DECL_OVERRIDE;
    void exposeEvent(QExposeEvent* event) Q_DECL_OVERRIDE;
//    QOpenGLPaintDevice* getPaintDevice();

private:
    bool m_update_pending;
    bool m_animating;

    QOpenGLPaintDevice* paintDevice;
    QOpenGLContext* glContext;
};

#endif // GL_WINDOW_H
