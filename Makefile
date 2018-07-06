CC = g++
CFLAGS = -Wall -pedantic
CXXFLAGS = -std=c++14 -isystem /usr/local/include/eigen3/
LDFLAGS =

BINARY = mview
OBJ = main.o \
      match.o \
      align.o \
	  ssd.o \
	  write_mesh.o \
	  read_dataset.o \
	  read_image.o \
	  rectify.o \
		triangulate.o \

LDLIBS = freeimage opencv_core opencv_calib3d opencv_imgproc

all: $(BINARY)

$(BINARY): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) $(addprefix -l,$(LDLIBS)) -o $(BINARY)

clean:
	rm -f $(OBJ)
	rm -f $(BINARY)

%.o: %.cpp
	$(CC) -c $(CFLAGS) $(CXXFLAGS) $< -o $@

