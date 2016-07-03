#include <QApplication>
#include <QSurfaceFormat>
#include <QGridLayout>
#include <QPushButton>

#include "window/main_window.h"
#include "window/gl_window.h"
#include "window/simulation_window.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

//    QMainWindow mainWindow;
//    mainWindow.resize(1000, 800);
//    mainWindow.show();

    QSurfaceFormat format;
    format.setSamples(16);

    SimulationWindow simWindow;
    simWindow.setFormat(format);
    simWindow.resize(720, 540);
    simWindow.show();
    simWindow.setAnimating(true);

//    QWidget* simWindowContainer = QWidget::createWindowContainer(&simWindow);
//    simWindowContainer->setMinimumSize(simWindow.size());
//    mainWindow.layout()->addWidget(simWindowContainer);
//    //mainWindow.centralWidget()->layout()->addWidget(simWindowContainer);

    return app.exec();
}
