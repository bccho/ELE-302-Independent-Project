TARGET=pingpong
CC=g++
LIBS=-lwiringPi -larmadillo
CFLAGS=-I.
OPS=-O2
DEPS=Pixy.h timer.h

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: $(TARGET).o
	$(CC) -o $(TARGET) $(TARGET).o $(OPS) $(LIBS)

clean:
	rm -f *.o $(TARGET)
