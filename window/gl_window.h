#ifndef GLWINDOW_H
#define GLWINDOW_H

#include <QOpenGLFunctions>
#include <QOpenGLPaintDevice>
#include <QWindow>
#include <QMainWindow>

//namespace Ui {
//    class MainWindow;
//}

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

private:
    //Ui::MainWindow* ui;

    bool m_update_pending;
    bool m_animating;

    QOpenGLContext* glContext;
    QOpenGLPaintDevice* paintDevice;
};

#endif // GLWINDOW_H
