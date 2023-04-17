#include "CSRMatrixHelper.h"
#include "Laplacian.h"

inline int LinearIndex(const int i, const int j, const int k)
{ return ((i * YDIM) + j) * ZDIM + k; }

CSRMatrix BuildUpperTriangularLaplacianMatrix()
{
    static constexpr int matSize = XDIM * YDIM * ZDIM;
    CSRMatrixHelper matrixHelper(matSize);
    
    for (int i = 1; i < XDIM-1; i++)
    for (int j = 1; j < YDIM-1; j++)
    for (int k = 1; k < ZDIM-1; k++) {
        matrixHelper( LinearIndex(i, j, k), LinearIndex(i, j, k) ) = 6.;
        if (i < XDIM-2)
            matrixHelper( LinearIndex(i, j, k), LinearIndex(i+1, j, k) ) = -1.;
        if (j < YDIM-2)
            matrixHelper( LinearIndex(i, j, k), LinearIndex(i, j+1, k) ) = -1.;
        if (k < ZDIM-2)
            matrixHelper( LinearIndex(i, j, k), LinearIndex(i, j, k+1) ) = -1.;
    }
    
    // Need to put part of the "identity" matrix on nodes of the margin
    for (int i = 0; i < XDIM; i++)
    for (int j = 0; j < YDIM; j++)
    for (int k = 0; k < ZDIM; k++)
        if ( i == 0 || i == XDIM-1 || j == 0 || j == YDIM-1 || k == 0 || k == ZDIM-1 )
            matrixHelper( LinearIndex(i, j, k), LinearIndex(i, j, k) ) = 1.;

    return matrixHelper.ConvertToCSRMatrix();
}
