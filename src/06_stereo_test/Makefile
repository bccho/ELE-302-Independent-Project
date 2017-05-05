CC=g++
LIBS=-lwiringPi
CFLAGS=-I.
DEPS = Pixy.h timer.h

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

stereo: stereo.o
	$(CC) -o stereo stereo.o $(LIBS)

clean:
	rm -f *.o stereo
