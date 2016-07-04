include ../../../../Makefile.conf

MODULE_NAME = OpenTLD/src/opentld
MODULE_COMMENT = OpenTLD/src/opentld

LINK = g++

CXXFLAGS += 
CFLAGS += 
IFLAGS += `pkg-config --cflags opencv` -I ../3rdparty/cvblobs -I ../libopentld/mftracker -I ../libopentld/tld -I ../libopentld/imacq -I main -I ../3rdparty/libconfig 
LFLAGS += `pkg-config --libs opencv` -lopentld  -l3rdparty -lopencv_video -lparam_interface -lipc -lglobal -lbumblebee_basic_interface
SUBDIRS += 

SOURCES =  tracker_TLD_main.cpp \
	main/Config.cpp  \
    main/Gui.cpp \
    main/Settings.cpp \
	main/Trajectory.cpp
    
PUBLIC_INCLUDES = 
    
PUBLIC_LIBRARIES = 
TARGETS = tracker_TLD
PUBLIC_BINARIES =  

tracker_TLD: tracker_TLD_main.o \
	main/Config.o  \
    main/Gui.o \
    main/Settings.o \
	main/Trajectory.o 

include ../../../../Makefile.rules
