#pragma once

#include "Parameters.h"
#include "CSRMatrix.h"

void DirectSparseSolver(
    CSRMatrix& matrix,
    float (&x)[XDIM][YDIM][ZDIM],
    float (&f)[XDIM][YDIM][ZDIM],
    const bool writeOutput = true);

