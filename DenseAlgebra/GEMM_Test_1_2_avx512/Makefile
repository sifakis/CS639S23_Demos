all: main.cpp Utilities.cpp MatMatMultiply.cpp MatMatMultiplyBlockHelper.o
	icc main.cpp Utilities.cpp MatMatMultiply.cpp MatMatMultiplyBlockHelper.o -xCOMMON-AVX512 -mkl -qopenmp

MatMatMultiplyBlockHelper.o: MatMatMultiplyBlockHelper.cpp
	icc -c MatMatMultiplyBlockHelper.cpp -xCOMMON-AVX512 -fno-alias

assembly: MatMatMultiplyBlockHelper.cpp
	icc -S MatMatMultiplyBlockHelper.cpp -xCOMMON-AVX512 -fno-alias

clean:
	rm a.out *.s *.o

