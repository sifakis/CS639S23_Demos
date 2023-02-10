#pragma once

#include "CSRMatrix.h"
#include "Parameters.h"

CSRMatrix BuildLaplacianMatrix();
CSRMatrix BuildLaplacianMatrixNoBoundary();

void ComputeLaplacian(CSRMatrix& laplacianMatrix,
    const float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM], bool usingTranspose=false);
