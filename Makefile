CC = g++
CFLAGS = -Wall -pedantic
CXXFLAGS = -std=c++14 -isystem /usr/include/eigen3
LDFLAGS =

BINARY = mview
OBJ = main.o \
      match.o \
      align.o \
	  ssd.o \
	  write_mesh.o \
	  read_image.o \
	  read_dataset.o \
	  stub.o \

LDLIBS = freeimage

all: $(BINARY)

$(BINARY): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) $(addprefix -l,$(LDLIBS)) -o $(BINARY)

clean:
	rm $(OBJ)
	rm $(BINARY)

%.o: %.cpp
	$(CC) -c $(CFLAGS) $(CXXFLAGS) $< -o $@
