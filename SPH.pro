#-------------------------------------------------
#
# Project created by QtCreator 2016-04-23T08:22:17
#
#-------------------------------------------------

QT += core gui opengl
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SPH
TEMPLATE = app
CONFIG += mobility c++11
QMAKE_CXXFLAGS += -std=c++11 -pthread -fopenmp
#QMAKE_LFLAGS +=
MOBILITY =

LIBS += -pthread -fopenmp

# Defines platform-specific preprocessor macro
#   edit at Projects->QMake->additional arguments
#   (set to CONFIG+=_BUILD) required first
CONFIG(DESKTOP_BUILD): DEFINES += DESKTOP_BUILD
CONFIG(ANDROID_BUILD): DEFINES += ANDROID_BUILD

RESOURCES += \
    D:/Dropbox/well/WFAIS/prog/Qt/SPH

############################################################
# Project files ############################################
############################################################

FORMS += \
    window/main_window.ui

DISTFILES += \
    shader/vertex_shader.vsh \
    shader/fragment_shader.fsh \
    sph.qmodel

HEADERS += \
    control/interaction.h \
    gl/form.h \
    gl/handles.h \
    gl/matrices.h \
    gl/vertex_array.h \
    machine/cloth.h \
    machine/machine.h \
    machine/particle.h \
    machine/spring.h \
    machine/rope.h \
    machine/walls.h \
    physics/computer.h \
    physics/forces.h \
    physics/geometry.h \
    physics/grid.h \
    shader/shader.h \
    physics/vector.h \
    shape/arrow.h \
    shape/dot.h \
    shape/landschaft.h \
    shape/line.h \
    shape/rectangle.h \
    shape/shape.h \
    shape/sphere.h \
    shape/triangle.h \
    util/debug_helper.h \
    util/enums.h \
    util/constants.h \
    util/operations.h \
    util/parser.h \
    util/settings.h \
    util/timer.h \
    window/gl_window.h \
    window/simulation_window.h \
    window/main_window.h

SOURCES += \
    control/interaction.cpp \
    gl/form.cpp \
    gl/handles.cpp \
    gl/matrices.cpp \
    gl/vertex_array.cpp \
    machine/cloth.cpp \
    machine/machine.cpp \
    machine/particle.cpp \
    machine/rope.cpp \
    machine/spring.cpp \
    machine/walls.cpp \
    main.cpp \
    physics/computer.cpp \
    physics/forces.cpp \
    physics/grid.cpp \
    physics/vector.cpp \
    shader/shader.cpp \
    shape/arrow.cpp \
    shape/dot.cpp \
    shape/landschaft.cpp \
    shape/line.cpp \
    shape/rectangle.cpp \
    shape/shape.cpp \
    shape/sphere.cpp \
    shape/triangle.cpp \
    util/debug_helper.cpp \
    util/enums.cpp \
    util/settings.cpp \
    util/timer.cpp \
    window/gl_window.cpp \
    window/simulation_window.cpp \
    window/main_window.cpp
