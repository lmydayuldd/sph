#include <QApplication>
#include <QSurfaceFormat>

#include "window/gl_window.h"
#include "window/simulation_window.h"

#include "gtest/gtest.h"
#include "gmock/gmock.h"

int main(int argc, char* argv[])
{
    QApplication app(argc, argv);

    //::testing::InitGoogleMock(&argc, argv);
    ::testing::InitGoogleTest(&argc, argv);

//    QSurfaceFormat format;
//    format.setSamples(16);

//    SimulationWindow simWindow;
//    simWindow.setFormat(format);
//    simWindow.resize(720, 540);
//    simWindow.show();

//    simWindow.setAnimating(true);

//    return app.exec();

    return RUN_ALL_TESTS();
}
