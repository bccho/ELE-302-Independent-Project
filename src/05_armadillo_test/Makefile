CC=g++
LIBS=-lwiringPi -larmadillo
CFLAGS=-I.
OPS=-O2
DEPS=

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

test: test.o
	$(CC) -o test test.o $(OPS) $(LIBS)

clean:
	rm -f *.o test
