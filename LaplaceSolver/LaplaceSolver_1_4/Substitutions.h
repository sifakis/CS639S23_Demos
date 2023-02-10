#pragma once

#include "CSRMatrix.h"

void ForwardSubstitution(CSRMatrix& L, float *x);
void BackwardSubstitution(CSRMatrix& Ut, float *x); // Matrix is presumed upper-triangular in CSC format, but what is actually
                                                    // passed in is the *transpose*, which is a lower-triangular CSR matrix!

