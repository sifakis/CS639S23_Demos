#include "ConjugateGradients.h"
#include "Laplacian.h"
#include "PointwiseOps.h"
#include "Reductions.h"
#include "Utilities.h"
#include "Timer.h"

#include <iostream>

extern Timer timerLaplacian;

void ConjugateGradients(
    CSRMatrix& matrix1,
    CSRMatrix& matrix2,
    float (&x)[XDIM][YDIM][ZDIM],
    const float (&f)[XDIM][YDIM][ZDIM],
    float (&p)[XDIM][YDIM][ZDIM],
    float (&r)[XDIM][YDIM][ZDIM],
    float (&z)[XDIM][YDIM][ZDIM],
    const bool writeIterations)
{
    // Algorithm : Line 2
    timerLaplacian.Restart(); ComputeLaplacian(matrix1, x, z); timerLaplacian.Pause();
    Saxpy(z, f, r, -1);
    float nu = Norm(r);

    // Algorithm : Line 3
    if (nu < nuMax) return;
        
    // Algorithm : Line 4
    Copy(r, p);
    float rho=InnerProduct(p, r);
        
    // Beginning of loop from Line 5
    for(int k=0;;k++)
    {
        std::cout << "Residual norm (nu) after " << k << " iterations = " << nu << std::endl;

        // Algorithm : Line 6
        timerLaplacian.Restart(); ComputeLaplacian(matrix2, p, z, true); timerLaplacian.Pause();
        float sigma=InnerProduct(p, z);

        // Algorithm : Line 7
        float alpha=rho/sigma;

        // Algorithm : Line 8
        Saxpy(z, r, r, -alpha);
        nu=Norm(r);

        // Algorithm : Lines 9-12
        if (nu < nuMax || k == kMax) {
            Saxpy(p, x, x, alpha);
            std::cout << "Conjugate Gradients terminated after " << k << " iterations; residual norm (nu) = " << nu << std::endl;
            if (writeIterations) WriteAsImage("x", x, k, 0, 127);
            return;
        }
            
        // Algorithm : Line 13
        Copy(r, z);
        float rho_new = InnerProduct(z, r);

        // Algorithm : Line 14
        float beta = rho_new/rho;

        // Algorithm : Line 15
        rho=rho_new;

        // Algorithm : Line 16
        Saxpy(p, x, x, alpha);
        Saxpy(p, r, p, beta);

        if (writeIterations) WriteAsImage("x", x, k, 0, 127);
    }

}
