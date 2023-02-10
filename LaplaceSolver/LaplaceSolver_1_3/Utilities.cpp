#include "Utilities.h"

#include <fstream>
#include <sstream>
#include <iostream>
#include <iomanip>
#include <stdexcept>

void Clear(float (&x)[XDIM][YDIM][ZDIM])
{
#pragma omp parallel for
    for (int i = 0; i < XDIM; i++)
    for (int j = 0; j < YDIM; j++)
    for (int k = 0; k < ZDIM; k++)
        x[i][j][k] = 0.;
}

void InitializeProblem(float (&x)[XDIM][YDIM][ZDIM], float (&b)[XDIM][YDIM][ZDIM]){

    // Start by zeroing out x and b
    Clear(x);
    Clear(b);

    // Make some of the boundary of values of x non-zero
    // (this operation is far too simple to be worth parallelizing)

    for(int i = XDIM/4; i < 3*(XDIM/4); i++)
    for(int j = XDIM/4; j < 3*(XDIM/4); j++)
        x[i][j][0] = 1.;
}

void WriteAsImage(const std::string& filenamePrefix, const float (&x)[XDIM][YDIM][ZDIM], const int count, const int axis, const int slice)
{
    std::ostringstream filename;
    filename << filenamePrefix << "." << std::setfill('0') << std::setw(4) << count << ".pgm";
    
    std::ofstream output(filename.str());

    output << "P2" << std::endl;
    switch(axis){
        case 0: output << YDIM << " " << ZDIM << std::endl; break;
        case 1: output << XDIM << " " << ZDIM << std::endl; break;
        case 2: output << XDIM << " " << YDIM << std::endl; break;
        default: throw std::logic_error("Invalid axis in WriteAsImage()");
    }
    output << "255" << std::endl;

    switch(axis){
        
        case 0:
            for (int j = 0; j < YDIM; j++){
                for (int k = 0; k < ZDIM; k++)
                    output << (int)(x[slice][j][k]*255.0) << " ";
                output << std::endl;
            }
            break;
        case 1:
            for (int i = 0; i < XDIM; i++){
                for (int k = 0; k < ZDIM; k++)
                    output << (int)(x[i][slice][k]*255.0) << " ";
                output << std::endl;
            }
            break;
        case 2:
            for (int i = 0; i < XDIM; i++){
                for (int j = 0; j < YDIM; j++)
                    output << (int)(x[i][j][slice]*255.0) << " ";
                output << std::endl;
            }
            break;
        default: throw std::logic_error("Invalid axis in WriteAsImage()");
    }
    
    output.close();
}
