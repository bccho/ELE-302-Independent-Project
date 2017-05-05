TARGET=kinematics
CC=g++
LIBS=-lwiringPi
CFLAGS=-I.
DEPS=Pixy.h timer.h

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: $(TARGET).o
	$(CC) -o $(TARGET) $(TARGET).o $(LIBS)

clean:
	rm -f *.o $(TARGET)
