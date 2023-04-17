all: main.cpp Utilities.cpp MatMatMultiply.cpp MatMatMultiplyBlockHelper.o
	icc main.cpp Utilities.cpp MatMatMultiply.cpp MatMatMultiplyBlockHelper.o -xHost -mkl -qopenmp

MatMatMultiplyBlockHelper.o: MatMatMultiplyBlockHelper.cpp
	icc -c MatMatMultiplyBlockHelper.cpp -xHost -qopenmp -fno-alias

assembly: MatMatMultiplyBlockHelper.cpp
	icc -S MatMatMultiplyBlockHelper.cpp -xHost -qopenmp -fno-alias

clean:
	rm a.out *.s *.o

