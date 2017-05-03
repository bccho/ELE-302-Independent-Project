CC=g++
LIBS=-lwiringPi -larmadillo
CFLAGS=-I.
DEPS =

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

test: test.o
	$(CC) -o test test.o $(LIBS)

clean:
	rm -f *.o test
