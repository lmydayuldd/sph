#-------------------------------------------------
#
# Project created by QtCreator 2016-04-23T08:22:17
#
#-------------------------------------------------

QT += core gui
greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = SPH
TEMPLATE = app
CONFIG += mobility c++11
MOBILITY =

FORMS +=

# Defines platform-specific preprocessor macro (edit at Projects->QMake arguments required first)
CONFIG(DESKTOP_BUILD):DEFINES += DESKTOP_BUILD
CONFIG(ANDROID_BUILD):DEFINES += ANDROID_BUILD

RESOURCES += \
    D:/Dropbox/well/WFAIS/prog/Qt/SPH

DISTFILES += \
    shader/vertex_shader.vsh \
    shader/fragment_shader.fsh \
    sph.qmodel

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
    util/enums.cpp \
    util/constants.cpp \
    util/settings.cpp \
    window/gl_window.cpp \
    window/simulation_window.cpp

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
    physics/geometry.h \
    physics/forces.h \
    physics/computer.h \
    physics/vector.h \
    shader/shader.h \
    shape/arrow.h \
    shape/dot.h \
    shape/landschaft.h \
    shape/line.h \
    shape/rectangle.h \
    shape/shape.h \
    shape/sphere.h \
    shape/triangle.h \
    util/enums.h \
    util/constants.h \
    util/operations.h \
    util/parser.h \
    util/settings.h \
    util/timer.h \
    window/gl_window.h \
    window/simulation_window.h
