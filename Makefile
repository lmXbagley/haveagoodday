CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
X11FLAGS := $(shell pkg-config --cflags --libs x11 2>/dev/null || echo '-lX11')
LIBS = $(X11FLAGS) -lpthread

all: organoid-sim drive

organoid-sim: $(wildcard src/*.cpp) $(wildcard src/*.hpp)
	$(CXX) $(CXXFLAGS) $(wildcard src/*.cpp) -o $@ $(LIBS)

drive: client/drive.cpp
	$(CXX) $(CXXFLAGS) client/drive.cpp -o $@ $(LIBS)

clean:
	rm -f organoid-sim drive

.PHONY: all clean
