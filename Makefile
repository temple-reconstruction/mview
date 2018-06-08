CC = g++
CFLAGS = -Wall -pedantic
CPPFLAGS = -std=c++14

BINARY = mview
OBJ = main.o
LIBRARIES = -lfreeimage

all: $(OBJ)
	$(CC) $(CFLAGS) -o $(BINARY) $(OBJ) $(LIBRARIES)

clean:
	rm $(OBJ)
	rm mview

%.o: %.cpp
	$(CC) -c $(CFLAGS) $(CPPFLAGS) $< -o $@
