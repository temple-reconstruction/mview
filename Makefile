CC = g++
CFLAGS = -Wall -pedantic
CXXFLAGS = -std=c++14 -I/usr/include/eigen3
LDFLAGS =

BINARY = mview
OBJ = main.o \
      match.o \
      align.o \

LDLIBS = freeimage

all: $(BINARY)

$(BINARY): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) $(addprefix -l,$(LDLIBS)) -o $(BINARY)

clean:
	rm $(OBJ)
	rm $(BINARY)

%.o: %.cpp
	$(CC) -c $(CFLAGS) $(CXXFLAGS) $< -o $@
