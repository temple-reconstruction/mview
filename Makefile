CC = g++
CFLAGS = -Wall -pedantic -O3
CXXFLAGS = -std=c++14 -isystem /usr/include/eigen3/
LDFLAGS =

BINARY = mview
OBJ = main.o \
      matcher.o \
      match_cv.o \
      align.o \
      ssd.o \
      sdf.o \
      write_mesh.o \
      rectify.o \
      triangulate.o \
      read_synthetic.o \
      # read_dataset.o read_image.o \

LDLIBS = freeimage opencv_core opencv_calib3d opencv_imgproc opencv_highgui opencv_imgcodecs

all: $(BINARY)

$(BINARY): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) $(addprefix -l,$(LDLIBS)) -o $(BINARY)

clean:
	rm -f $(OBJ)
	rm -f $(BINARY)

%.o: %.cpp mview.h
	$(CC) -c $(CFLAGS) $(CXXFLAGS) $< -o $@

