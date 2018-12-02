CC=g++
CFLAGS=-Wall -std=c++11
MAINBIN=GVRP
MAINOBJ=main.o Model.o


all: $(MAINBIN)

$(MAINBIN): $(MAINOBJ)
	$(CC) $(CFLAGS) -o $@ $^

main.o: main.cpp Model.hpp
	$(CC) $(CFLAGS) -c $<

Model.o: Model.cpp Model.hpp
	$(CC) $(CFLAGS) -c $<
.PHONY : clean
clean: 
	rm -f *.o Model main