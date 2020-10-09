TEMPLATE = app
CONFIG += console
CONFIG -= qt
CONFIG += console c++11
CONFIG -= app_bundle
QMAKE_CXXFLAGS += -std=gnu++14
QMAKE_CFLAGS_RELEASE += -fopenmp
QMAKE_CXXFLAGS += -fopenmp
QMAKE_LFLAGS += -fopenmp


SOURCES += \
    src/Heuristics/BriefHeuristic.cpp \
    src/Heuristics/Heuristic.cpp \
    src/Map/Grid.cpp \
    src/Map/MapGrid.cpp \
    src/Mcl/Mcl.cpp \
    src/Robot/DroneRobot.cpp \
    src/Robot/Robot.cpp \
    src/Utils/ColorCPU.cpp \
    src/Utils/GlutClass.cpp \
    src/Utils/Utils.cpp \
    src/Utils/vec3.cpp \
    src/VegetationIndex/Grvi.cpp \
    src/VegetationIndex/Ndvi.cpp \
    src/VegetationIndex/VegetationIndex.cpp \
    src/VegetationIndex/VegetatonIndexGen.cpp \
    main.cpp

HEADERS += \
    src/Heuristics/BriefHeuristic.h \
    src/Heuristics/Heuristic.h \
    src/Map/Grid.h \
    src/Map/MapGrid.h \
    src/Mcl/Mcl.h \
    src/Robot/DroneRobot.h \
    src/Robot/Robot.h \
    src/Utils/ColorCPU.h \
    src/Utils/GlutClass.h \
    src/Utils/RadiusVolumeTransferFunctions.h \
    src/Utils/Utils.h \
    src/Utils/mat3x3.h \
    src/Utils/vec3.h \
    src/VegetationIndex/Grvi.h \
    src/VegetationIndex/Ndvi.h \
    src/VegetationIndex/VegetationIndex.h \
    src/VegetationIndex/VegetatonIndexGen.h \
    config.h

INCLUDEPATH += -I/usr/local/include/opencv2

LIBS += -L/usr/local/lib -lpthread -lglut -lGLEW -ldl -lrt `pkg-config opencv --libs` -lGL -lfreeimage -lboost_system -lopencv_ximgproc
LIBS += -lboost_system

INCLUDEPATH += $$PWD/../../../../../usr/local/include
DEPENDPATH += $$PWD/../../../../../usr/local/include

win32:CONFIG(release, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/release/ -lopencv_xfeatures2d
else:win32:CONFIG(debug, debug|release): LIBS += -L$$PWD/../../../../../usr/local/lib/debug/ -lopencv_xfeatures2d
else:unix: LIBS += -L$$PWD/../../../../../usr/local/lib/ -lopencv_xfeatures2d
