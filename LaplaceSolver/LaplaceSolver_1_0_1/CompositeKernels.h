#pragma once

#include "CSRMatrix.h"
#include "Parameters.h"

float ComputeLaplacianAndInnerProduct(CSRMatrix& laplacianMatrix,
    const float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM])
{
    int N = laplacianMatrix.mSize;
    const auto rowOffsets = laplacianMatrix.GetRowOffsets();
    const auto columnIndices = laplacianMatrix.GetColumnIndices();
    const auto values = laplacianMatrix.GetValues();
    float *y = &Lu[0][0][0];
    const float *x = &u[0][0][0];
    

    double result = 0.;
#pragma omp parallel for reduction(+:result)
    for (int i = 0; i < N; i++)
    {
        y[i] = 0.;
        for (int k = rowOffsets[i]; k < rowOffsets[i+1]; k++) {
            const int j = columnIndices[k];
            y[i] += values[k] * x[j];
        }
        result += (double) x[i] * (double) y[i];
    }
    
    return (float) result;
}
