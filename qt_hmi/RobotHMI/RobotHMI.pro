QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++17

# The following define makes your compiler emit warnings if you use
# any Qt feature that has been marked deprecated (the exact warnings
# depend on your compiler). Please consult the documentation of the
# deprecated API in order to know how to port your code away from it.
DEFINES += QT_DEPRECATED_WARNINGS

INCLUDEPATH += $$PWD/../../shared/include/

# You can also make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
# You can also select to disable deprecated APIs only up to a certain version of Qt.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
    src/main.cpp \
    src/mainwindow.cpp \

HEADERS += \
    src/mainwindow.h \

FORMS += \
    src/mainwindow.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target


win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../install/lib/release/ -lshared_memory
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../install/lib/debug/ -lshared_memory
else:unix: LIBS += -L$$PWD/../../install/lib/ -lshared_memory

INCLUDEPATH += $$PWD/../../install
DEPENDPATH += $$PWD/../../install

win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../install/lib/release/libshared_memory.a
else:win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../install/lib/debug/libshared_memory.a
else:win32:!win32-g++:CONFIG(release, debug|release): PRE_TARGETDEPS += $$PWD/../../install/lib/release/shared_memory.lib
else:win32:!win32-g++:CONFIG(debug, debug|release): PRE_TARGETDEPS += $$PWD/../../install/lib/debug/shared_memory.lib
else:unix: PRE_TARGETDEPS += $$PWD/../../install/lib/libshared_memory.a
