#include "MatVecMultiply.h"

// #define DO_NOT_USE_MKL
#ifndef DO_NOT_USE_MKL
#include <mkl.h>
#endif

void MatVecMultiply(CSRMatrix& mat, const float *x, float *y)
{
    int N = mat.mSize;
    const auto rowOffsets = mat.GetRowOffsets();
    const auto columnIndices = mat.GetColumnIndices();
    const auto values = mat.GetValues();

#ifdef DO_NOT_USE_MKL
#pragma omp parallel for
    for (int i = 0; i < N; i++)
    {
        y[i] = 0.;
        for (int k = rowOffsets[i]; k < rowOffsets[i+1]; k++) {
            const int j = columnIndices[k];
            y[i] += values[k] * x[j];
        }
    }
#else
    mkl_cspblas_scsrgemv( // (S)parse (CSR) (Ge)neral matrix (M)atrix-(V)ector product
        "N",              // Use the normal matrix, not its transpose
        &N,               // Size of the matrix
        values,           // values array (MKL denotes this as "a")
        rowOffsets,       // rowOffsets array (MKL denotes this as "ia")
        columnIndices,    // columnIndices array (MKL denotes this as "ja")
        x,                // Vector getting multiplied
        y                 // Vector where the product gets stored
    );
#endif
}
