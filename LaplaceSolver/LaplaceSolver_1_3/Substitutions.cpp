#include "Substitutions.h"

void ForwardSubstitution(CSRMatrix& L, float *x) { // In-place

    const auto N = L.mSize;
    const auto rowOffsets = L.GetRowOffsets();
    const auto columnIndices = L.GetColumnIndices();
    const auto values = L.GetValues();
    
    for (int i = 0; i < N; i++) {
        for (int k = rowOffsets[i]; k < rowOffsets[i+1]-1; k++)
            x[i] -= values[k] * x[columnIndices[k]]; // Move all terms of the i-th equation
                                                     // involving x[j]'s (with j<i) to the right
                                                     // hand side (which is stored in x[i] 
        x[i] /= values[rowOffsets[i+1]-1]; // Divide by the diagonal matrix entry
    }
}

void BackwardSubstitution(CSRMatrix& Ut, float *x) { // In-place

    const auto N = Ut.mSize;
    const auto columnOffsets = Ut.GetRowOffsets();
    const auto rowIndices = Ut.GetColumnIndices();
    const auto values = Ut.GetValues();
    
    for (int j = N-1; j >= 0; j--) {
        x[j] /= values[columnOffsets[j+1]-1]; // Divide by the diagonal matrix entry
        for (int k = columnOffsets[j+1]-2; k >= columnOffsets[j]; k--)
            x[rowIndices[k]] -= values[k] * x[j]; // Annihilate the value of x[j] from all equations
                                                  // (indexed lower than the j-th equation) where such
                                                  // a term exists. The right hand side of the i-th
                                                  // equation is stored in x[i]
    }
}
