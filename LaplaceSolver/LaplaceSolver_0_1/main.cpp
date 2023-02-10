#include "Timer.h"

#include "Laplacian.h"
#include "Utilities.h"
#include "PointwiseOps.h"
#include "Reductions.h"


int main(int argc, char *argv[])
{
    using array_t = float (&) [XDIM][YDIM][ZDIM];

    float *xRaw = new float [XDIM*YDIM*ZDIM];
    float *fRaw = new float [XDIM*YDIM*ZDIM];
    float *pRaw = new float [XDIM*YDIM*ZDIM];
    float *rRaw = new float [XDIM*YDIM*ZDIM];
    float *zRaw = new float [XDIM*YDIM*ZDIM];
    
    array_t x = reinterpret_cast<array_t>(*xRaw);
    array_t f = reinterpret_cast<array_t>(*fRaw);
    array_t p = reinterpret_cast<array_t>(*pRaw);
    array_t r = reinterpret_cast<array_t>(*rRaw);
    array_t z = reinterpret_cast<array_t>(*zRaw);

    Timer timer;
    
    // Initialization
    {
        timer.Start();
        InitializeProblem(x, f);
        WriteAsImage("x", x, 0, 0, 127);
        Clear(p);
        Clear(r);
        Clear(z);
        timer.Stop("Initialization : ");
    }

    // Conjugate Gradients iteration
    {
        ComputeLaplacian(x, z);
        Saxpy(z, f, r, -1);
        Copy(r, p);
        float rho=InnerProduct(r,r);
        float rho_old, convergence_norm=0;
        
        for(int iterations=0;;iterations++)
        {
            convergence_norm=Norm(r);
            std::cout << "Norm = " << convergence_norm << std::endl;
            
            rho=InnerProduct(r,r);
            if(iterations>0)
                Saxpy(p, r, p, rho/rho_old);
            
            ComputeLaplacian(p, z);
            float p_dot_z=InnerProduct(p, z);
            float alpha=rho/p_dot_z;
            Saxpy(p, x, x, alpha);
            Saxpy(z, r, r, -alpha);
            rho_old=rho;
            WriteAsImage("x", x, iterations, 0, 127);
        }
    }

    return 0;
}
