#pragma once

#include "Parameters.h"

// Compute the maximum absolute value among the array elements
float Norm(const float (&x)[XDIM][YDIM][ZDIM]);

// Compute the "dot product" between the two arrays
float InnerProduct(const float (&x)[XDIM][YDIM][ZDIM], const float (&y)[XDIM][YDIM][ZDIM]);
