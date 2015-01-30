CC = g++
CFLAGS = -std=c++11 -g -Wall -fdiagnostics-color=auto
SRCS = calibrate.cpp
PROG = calibrate

OPENCV = `pkg-config opencv --cflags --libs`
LIBS = $(OPENCV)


$(PROG):$(SRCS)
	$(CC) $(CFLAGS)  -o $(PROG) $(SRCS) $(LIBS) $(BOOST)

clean:
	rm $(PROG)
