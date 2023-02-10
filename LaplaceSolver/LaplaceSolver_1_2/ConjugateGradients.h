#pragma once

#include "Parameters.h"
#include "CSRMatrix.h"

void ConjugateGradients(
    CSRMatrix& matrix1,
    CSRMatrix& matrix2,
    float (&x)[XDIM][YDIM][ZDIM],
    const float (&f)[XDIM][YDIM][ZDIM],
    float (&p)[XDIM][YDIM][ZDIM],
    float (&r)[XDIM][YDIM][ZDIM],
    float (&z)[XDIM][YDIM][ZDIM],
    const bool writeIterations = true);

