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
QMAKE_CXXFLAGS += -std=c++11 -pthread
#QMAKE_CFLAGS += -std=c++11 -pthread
MOBILITY =

# Defines platform-specific preprocessor macro
#   edit at Projects->QMake->additional arguments (set to CONFIG+=_BUILD) required first
CONFIG(DESKTOP_BUILD): DEFINES += DESKTOP_BUILD
CONFIG(ANDROID_BUILD): DEFINES += ANDROID_BUILD

RESOURCES += \
    D:/Dropbox/well/WFAIS/prog/Qt/SPH

# Google Test / Mock:

#DEFINES += GTEST_LINKED_AS_SHARED_LIBRARY=1

INCLUDEPATH += \
    $$PWD/googletest-master/googletest/include \
    $$PWD/googletest-master/googlemock/include \
    $$PWD/googletest-master/googletest \
    $$PWD/googletest-master/googlemock

#HEADERS += \
#    $$PWD/googletest-master/googletest/include/gtest/gtest.h \
#    $$PWD/googletest-master/googlemock/include/gmock/gmock.h

#LIBS += \
#    -lpthread \
#    -L$$PWD/googletest-master/googletest/lib/.libs -lgtest \
#    -L$$PWD/googletest-master/googlemock/lib/.libs -lgmock

SOURCES += \
    googletest-master/googlemock/src/gmock-all.cc \
    googletest-master/googlemock/src/gmock-cardinalities.cc \
    googletest-master/googlemock/src/gmock-internal-utils.cc \
    googletest-master/googlemock/src/gmock-matchers.cc \
    googletest-master/googlemock/src/gmock-spec-builders.cc \
    googletest-master/googlemock/src/gmock.cc \
    #googletest-master/googlemock/src/gmock_main.cc \
    googletest-master/googletest/src/gtest-all.cc \
    googletest-master/googletest/src/gtest-death-test.cc \
    googletest-master/googletest/src/gtest-filepath.cc \
    googletest-master/googletest/src/gtest-internal-inl.h \
    googletest-master/googletest/src/gtest-port.cc \
    googletest-master/googletest/src/gtest-printers.cc \
    googletest-master/googletest/src/gtest-test-part.cc \
    googletest-master/googletest/src/gtest-typed-test.cc \
    googletest-master/googletest/src/gtest.cc \# \
    test/tests.cpp
    #googletest-master/googletest/src/gtest_main.cc

HEADERS += \
    googletest-master/googlemock/include/gmock/internal/custom/gmock-generated-actions.h \
    googletest-master/googlemock/include/gmock/internal/custom/gmock-generated-actions.h.pump \
    googletest-master/googlemock/include/gmock/internal/custom/gmock-matchers.h \
    googletest-master/googlemock/include/gmock/internal/custom/gmock-port.h \
    googletest-master/googlemock/include/gmock/internal/gmock-generated-internal-utils.h \
    googletest-master/googlemock/include/gmock/internal/gmock-generated-internal-utils.h.pump \
    googletest-master/googlemock/include/gmock/internal/gmock-internal-utils.h \
    googletest-master/googlemock/include/gmock/internal/gmock-port.h \
    googletest-master/googlemock/include/gmock/gmock-actions.h \
    googletest-master/googlemock/include/gmock/gmock-cardinalities.h \
    googletest-master/googlemock/include/gmock/gmock-generated-actions.h \
    googletest-master/googlemock/include/gmock/gmock-generated-actions.h.pump \
    googletest-master/googlemock/include/gmock/gmock-generated-function-mockers.h \
    googletest-master/googlemock/include/gmock/gmock-generated-function-mockers.h.pump \
    googletest-master/googlemock/include/gmock/gmock-generated-matchers.h \
    googletest-master/googlemock/include/gmock/gmock-generated-matchers.h.pump \
    googletest-master/googlemock/include/gmock/gmock-generated-nice-strict.h \
    googletest-master/googlemock/include/gmock/gmock-matchers.h \
    googletest-master/googlemock/include/gmock/gmock-more-actions.h \
    googletest-master/googlemock/include/gmock/gmock-more-matchers.h \
    googletest-master/googlemock/include/gmock/gmock-spec-builders.h \
    googletest-master/googlemock/include/gmock/gmock.h \
    googletest-master/googlemock/test/gmock_link_test.h \
    googletest-master/googletest/include/gtest/internal/custom/gtest-port.h \
    googletest-master/googletest/include/gtest/internal/custom/gtest-printers.h \
    googletest-master/googletest/include/gtest/internal/custom/gtest.h \
    googletest-master/googletest/include/gtest/internal/gtest-death-test-internal.h \
    googletest-master/googletest/include/gtest/internal/gtest-filepath.h \
    googletest-master/googletest/include/gtest/internal/gtest-internal.h \
    googletest-master/googletest/include/gtest/internal/gtest-linked_ptr.h \
    googletest-master/googletest/include/gtest/internal/gtest-param-util-generated.h \
    googletest-master/googletest/include/gtest/internal/gtest-param-util.h \
    googletest-master/googletest/include/gtest/internal/gtest-port-arch.h \
    googletest-master/googletest/include/gtest/internal/gtest-port.h \
    googletest-master/googletest/include/gtest/internal/gtest-string.h \
    googletest-master/googletest/include/gtest/internal/gtest-tuple.h \
    googletest-master/googletest/include/gtest/internal/gtest-tuple.h.pump \
    googletest-master/googletest/include/gtest/internal/gtest-type-util.h \
    googletest-master/googletest/include/gtest/gtest-death-test.h \
    googletest-master/googletest/include/gtest/gtest-message.h \
    googletest-master/googletest/include/gtest/gtest-param-test.h \
    googletest-master/googletest/include/gtest/gtest-param-test.h.pump \
    googletest-master/googletest/include/gtest/gtest-printers.h \
    googletest-master/googletest/include/gtest/gtest-spi.h \
    googletest-master/googletest/include/gtest/gtest-test-part.h \
    googletest-master/googletest/include/gtest/gtest-typed-test.h \
    googletest-master/googletest/include/gtest/gtest.h \
    googletest-master/googletest/include/gtest/gtest_pred_impl.h \
    googletest-master/googletest/include/gtest/gtest_prod.h \
    googletest-master/googletest/samples/prime_tables.h \
    googletest-master/googletest/samples/sample1.h \
    googletest-master/googletest/samples/sample2.h \
    googletest-master/googletest/samples/sample3-inl.h \
    googletest-master/googletest/samples/sample4.h \
    googletest-master/googletest/src/gtest-internal-inl.h \
    googletest-master/googletest/test/gtest-param-test_test.h \
    googletest-master/googletest/test/gtest-typed-test_test.h \
    googletest-master/googletest/test/production.h \
    test/tests.h \
    mock/mock_particle.h

# Project files:

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
    window/simulation_window.h \
    machine/obstacle.h

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
    util/settings.cpp \
    window/gl_window.cpp \
    window/simulation_window.cpp \
    machine/obstacle.cpp \
    util/timer.cpp \
    main_test.cpp

SUBDIRS += \
    SPH.pro
