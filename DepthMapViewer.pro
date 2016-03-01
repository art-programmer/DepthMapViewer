#-------------------------------------------------
#
# Project created by QtCreator 2015-06-11T14:13:19
#
#-------------------------------------------------

QT       += core gui opengl

TARGET = DepthMapViewer
TEMPLATE = app


SOURCES += \
    panorama.cc \
    utils.cc \
    view_parameters.cc \
    depth_map_renderer.cc \
    main_widget_renderer.cc \
    main_widget.cc \
    main.cc \
    poly2tri/common/shapes.cc \
    poly2tri/sweep/advancing_front.cc \
    poly2tri/sweep/cdt.cc \
    poly2tri/sweep/sweep.cc \
    poly2tri/sweep/sweep_context.cc \
    main_window.cpp

HEADERS += \
    panorama.h \
    utils.h \
    view_parameters.h \
    configuration.h \
    depth_map_renderer.h \
    file_io.h \
    main_widget.h \
    poly2tri/common/shapes.h \
    poly2tri/common/utils.h \
    poly2tri/sweep/advancing_front.h \
    poly2tri/sweep/cdt.h \
    poly2tri/sweep/sweep.h \
    poly2tri/sweep/sweep_context.h \
    poly2tri/poly2tri.h \
    main_window.h

INCLUDEPATH += /usr/local/include/

LIBS += -L/usr/local/lib
#LIBS += -lglut
# -lGLU
# -lGLEW
LIBS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs

mac: LIBS += -framework GLUT

CONFIG -= x86_64
