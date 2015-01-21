# should be either OSC_HOST_BIG_ENDIAN or OSC_HOST_LITTLE_ENDIAN
# Apple: OSC_HOST_BIG_ENDIAN
# Win32: OSC_HOST_LITTLE_ENDIAN
# i386 LinuX: OSC_HOST_LITTLE_ENDIAN

PLATFORM=$(shell uname)

SDL_CFLAGS  := $(shell sdl-config --cflags)
SDL_LDFLAGS := $(shell sdl-config --libs) -lpthread

PROG = captureStream

INCLUDES = -I./libbaumer/src/baumer/inc
OPENCV = `pkg-config opencv --cflags --libs`

LDPATH = -L/usr/local/lib/baumer

LDFLAGS = -lbgapi2_ext \
          -lbgapi2_genicam \
          -lbgapi2_img \
          -lcamera_tools \
          -levisionlib \
          -limage_tools \
          -lMathParser \

CFLAGS  = -fPIC -Wall -fdiagnostics-color=auto -O3 $(SDL_CFLAGS) -DLINUX -D_GNULINUX
CXXFLAGS = $(CFLAGS) $(INCLUDES) -std=c++11

SRC = calib.cpp
OBJ = calib.o

all: capturestream

capturestream: $(OBJ)
	$(CXX) -o $(PROG) $+ $(SDL_LDFLAGS) $(OPENCV) $(LDPATH) $(LDFLAGS)

clean:
	rm -rf $(PROG) $(OBJ)

clean-images:
	rm out/left/* && rm out/right/*

