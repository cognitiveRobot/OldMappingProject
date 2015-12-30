# This makefile builds all programs from PROGSRCS and links them with the source files from LIBSRCS.

CC=g++-4.4
CFLAGS=-c -O1 -Wall -fPIC -I/home/sarotti/GuidedMapping -I/usr/local/Aria/include -I/usr/local/include#-I/usr/local/include/opencv2 -I/usr/local/include
#CFLAGS += `pkg-config --cflags mrpt-base mrpt-slam mrpt-gui`
LDFLAGS=-L/usr/local/Aria/lib  -O1 -lAria -lpthread -ldl -lrt 
#LDFLAGS	+= `pkg-config --libs opencv mrpt-base mrpt-slam mrpt-gui`
LIBSRCS= \
	Object.cpp \
	Point.cpp \
	asr.cpp \
	GeometricOp.cpp \
	asrOp.cpp \
	PerceptualMapping.cpp \
	mfisOp.cpp \
	Mapping.cpp \
	readAndwriteASCII.cpp \
	Minfo.cpp \
	PointAndSurface.cpp \
	GeometryFuncs.cpp \
	GroupS.cpp  \
	Transporter.cpp \
	Plotting.cpp \
	PathPlanning.cpp \
	Map.cpp \
	CompareASR.cpp \
	CompareASRClass.cpp \
	Laser2Surface.cpp \
	Polygon.cpp  \
	PolygonChoice.cpp  \
	space.cpp  \
	display.cpp \
	convexPathPlanner.cpp \
	Comparison.cpp \
	thesis.cpp \
	clipper.cpp \
    
# These are the executable programs with a main() function
PROGSRCS= \
    mapWholeFloor.cpp \
    localmapAndglobal.cpp \
    

LIBOBJS = $(LIBSRCS:%.cpp=../obj/%.o)
PROGOBJS = $(PROGSRCS:%.cpp=../obj/%.o)
LIBDEPS = $(PROGOBJS:.o=.o.d)
PROGDEPS = $(LIBOBJS:.o=.o.d)

PROG = $(PROGSRCS:%.cpp=../bin/%)

all: $(LIBOBJS) $(PROGOBJS) $(PROG)

# Linking: for each executable, link its o file together with all the non-executable o files
../bin/%: $(LIBOBJS) $(PROGOBJS)
	$(CC) $(LDFLAGS) ../obj/$*.o $(LIBOBJS) -o ../bin/$*

# Compiling: first make dependencies, then compile all cpp files
../obj/%.o: %.cpp
	$(CC) $(CFLAGS) $< -MM -MT $@ -MF $@.d
	$(CC) $(CFLAGS) $< -o $@

clean:      ; rm $(LIBOBJS) $(PROGOBJS) $(LIBDEPS) $(PROGDEPS) $(PROG)

# Include the dependencies so the cpp files are recompiled if headers change
-include $(LIBDEPS)
-include $(PROGDEPS)
