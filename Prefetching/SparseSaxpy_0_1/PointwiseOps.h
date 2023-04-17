#pragma once

#include "Parameters.h"

#include <vector>

// Scale array x by given number, add y, and write result into y
void SparseSaxpy(std::vector<int>& blockOffsets, const float *x, float *y, const float scale);
