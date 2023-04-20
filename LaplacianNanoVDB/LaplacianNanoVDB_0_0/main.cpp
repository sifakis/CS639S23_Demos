#include "Timer.h"
#include "Laplacian.h"

#include <iomanip>

int main(int argc, char *argv[])
{
    using array_t = float (&) [XDIM][YDIM][ZDIM];
    
    float *uRaw = new float [XDIM*YDIM*ZDIM];
    float *LuRaw = new float [XDIM*YDIM*ZDIM];
    float *uVDB = new float [XDIM*YDIM*ZDIM];
    float *LuVDB = new float [XDIM*YDIM*ZDIM];
    uint32_t *flagsVDB = new uint32_t [XDIM*YDIM*ZDIM];
    array_t u = reinterpret_cast<array_t>(*uRaw);
    array_t Lu = reinterpret_cast<array_t>(*LuRaw);

    Timer timer;

    timer.Start();
    auto handle = initializeIndexGrid();
    timer.Stop("Initializing indexGrid - Elapsed time :");
    auto *indexGridPtr = handle.grid<nanovdb::ValueIndex>();

    timer.Start();
    initializeData(u, Lu, indexGridPtr, uVDB, LuVDB, flagsVDB);
    timer.Stop("Initializing data - Elapsed time :");
 
    std::cout << "Discrepancy between dense and VDB (u) = " << compareData(u, indexGridPtr, uVDB) << std::endl;

    for(int test = 1; test <= 10; test++)
    {
        std::cout << "Running test iteration (dense) " << std::setw(2) << test << " ";
        timer.Start();
        ComputeLaplacian(u, Lu);
        timer.Stop("Elapsed time : ");
    }
    
    for(int test = 1; test <= 10; test++)
    {
        std::cout << "Running test iteration (VDB) " << std::setw(2) << test << " ";
        timer.Start();
        computeLaplacianVDB(indexGridPtr, uVDB, LuVDB, flagsVDB);
        timer.Stop("Elapsed time : ");
    }
    
    std::cout << "Discrepancy between dense and VDB (Lu) = " << compareData(Lu, indexGridPtr, LuVDB) << std::endl;

    return 0;
}
