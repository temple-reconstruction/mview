CC = g++
CFLAGS = -Wall -pedantic
CPPFLAGS = -std=c++14 -isystem /usr/include/eigen3

BINARY = mview
OBJ = main.o \
      match.o \

LDLIBS = freeimage

all: $(BINARY)

$(BINARY): $(OBJ)
	$(CC) $(CFLAGS) $(LDFLAGS) $(OBJ) $(addprefix -l,$(LDLIBS)) -o $(BINARY)

clean:
	rm $(OBJ)
	rm $(BINARY)

%.o: %.cpp
	$(CC) -c $(CFLAGS) $(CXXFLAGS) $< -o $@
