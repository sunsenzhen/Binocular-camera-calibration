#-------------------------------------------------
#
# Project created by QtCreator 2017-10-30T23:24:39
#
#-------------------------------------------------

QT       += core

QT       -= gui

TARGET = grapImage
CONFIG   += console
CONFIG   -= app_bundle

TEMPLATE = app


SOURCES += \
    mainyuyvcamera-1.cpp \
    hvideocapture.cpp \
    sterocameratwo.cpp

unix: CONFIG += link_pkgconfig
unix: PKGCONFIG += /usr/local/lib/pkgconfig/opencv.pc

HEADERS += \
    hvideocapture.h \
    sterocameratwo.h
