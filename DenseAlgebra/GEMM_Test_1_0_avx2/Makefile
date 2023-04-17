all: main.cpp Utilities.cpp MatMatMultiply.cpp MatMatMultiplyBlockHelper.o
	icc main.cpp Utilities.cpp MatMatMultiply.cpp MatMatMultiplyBlockHelper.o -xavx2 -mkl -qopenmp

MatMatMultiplyBlockHelper.o: MatMatMultiplyBlockHelper.cpp
	icc -c MatMatMultiplyBlockHelper.cpp -xavx2 -fno-alias

assembly: MatMatMultiplyBlockHelper.cpp
	icc -S MatMatMultiplyBlockHelper.cpp -xavx2 -fno-alias

clean:
	rm a.out *.s *.o

