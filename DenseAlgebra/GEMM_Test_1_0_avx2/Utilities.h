#pragma once

#include "Parameters.h"

#include <cstdlib>

void* AlignedAllocate(const std::size_t size, const std::size_t alignment);
void InitializeMatrices(float (&A)[MATRIX_SIZE][MATRIX_SIZE],float (&B)[MATRIX_SIZE][MATRIX_SIZE]);
float MatrixMaxDifference(const float (&A)[MATRIX_SIZE][MATRIX_SIZE],const float (&B)[MATRIX_SIZE][MATRIX_SIZE]);
