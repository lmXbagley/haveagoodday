CXX ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
X11FLAGS := $(shell pkg-config --cflags --libs x11 2>/dev/null || echo '-lX11')
LIBS = $(X11FLAGS) -lpthread

SRC := $(wildcard src/*.cpp)
HDR := $(wildcard src/*.hpp)

organoid-sim: $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) $(SRC) -o $@ $(LIBS)

clean:
	rm -f organoid-sim

.PHONY: clean
