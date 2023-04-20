#include "Laplacian.h"

#include <nanovdb/util/GridBuilder.h>
#include <nanovdb/util/IndexGridBuilder.h>

void ComputeLaplacian(const float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM])
{

#pragma omp parallel for
    for (int i = 1; i < XDIM-1; i++)
    for (int j = 1; j < YDIM-1; j++)
    for (int k = 1; k < ZDIM-1; k++)
        Lu[i][j][k] =
            -6 * u[i][j][k]
            + u[i+1][j][k]
            + u[i-1][j][k]
            + u[i][j+1][k]
            + u[i][j-1][k]
            + u[i][j][k+1]
            + u[i][j][k-1];
            
}

nanovdb::GridHandle<nanovdb::HostBuffer>
initializeIndexGrid()
{
    nanovdb::GridBuilder<nanovdb::ValueMask> builder(true);
    auto acc = builder.getAccessor();
    for (int i = 0; i < XDIM; i++)
    for (int j = 0; j < YDIM; j++)
    for (int k = 0; k < ZDIM; k++) {
        nanovdb::Coord xyz(i,j,k);
        acc.setValue(xyz, true);
    }
    auto handle = builder.getHandle();
    auto *dstGrid = handle.grid<nanovdb::ValueMask>();
    nanovdb::IndexGridBuilder<nanovdb::ValueMask> indexBuilder(*dstGrid, false, false);
    return indexBuilder.getHandle();
}

void initializeData(float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM],
    nanovdb::NanoGrid<nanovdb::ValueIndex>* indexGridPtr,
    float *uBuffer, float *LuBuffer, uint32_t* flagsBuffer)
{
    float *uPtr = &u[0][0][0], *LuPtr = &Lu[0][0][0];

    // Zero out buffers by iterating linearly over them
    for (int n = 0; n < XDIM*YDIM*ZDIM; n++) {
        uPtr[n] = LuPtr[n] = uBuffer[n] = LuBuffer[n] = 0.0;
        flagsBuffer[n] = 0;
    }

    auto acc = indexGridPtr->getAccessor();
    for (int i = 1; i < XDIM-1; i++)
    for (int j = 1; j < YDIM-1; j++)
    for (int k = 1; k < ZDIM-1; k++) {
        nanovdb::Coord xyz(i,j,k);
        auto index = acc.getValue(xyz);
        u[i][j][k] = (float) ((i+j+k)%256-128);
        uBuffer[index] = u[i][j][k];
        flagsBuffer[index] = INDEX_ACTIVE_FLAG;
    }
}

float compareData(const float (&data)[XDIM][YDIM][ZDIM],
    nanovdb::NanoGrid<nanovdb::ValueIndex>* indexGridPtr,
    const float *dataVDBBuffer)
{
    float result = 0.;
    
    auto acc = indexGridPtr->getAccessor();
    for (int i = 0; i < XDIM; i++)
    for (int j = 0; j < YDIM; j++)
    for (int k = 0; k < ZDIM; k++) {
        nanovdb::Coord xyz(i,j,k);
        auto index = acc.getValue(xyz);
        result = std::max( result, std::abs(data[i][j][k]-dataVDBBuffer[index]) );
    }

    return result;
}

void computeLaplacianVDB(nanovdb::NanoGrid<nanovdb::ValueIndex>* indexGridPtr,
    float *uBuffer, float *LuBuffer, uint32_t* flagsBuffer)
{
    auto mgrHandle = createNodeManager(*indexGridPtr);
    auto *mgr = mgrHandle.template mgr<nanovdb::ValueIndex>();
    auto acc = indexGridPtr->getAccessor();

#pragma omp parallel for firstprivate(acc)
    for ( size_t l = 0; l < mgr->nodeCount(0); ++l ) // l enumerates "leaves"
        for( auto iter = mgr->leaf(l).beginValue(); iter; ++iter ){
            auto coord = iter.getCoord(); // this is the coordinate within the leaf
            auto indexCtr = *iter; // this is the "center" index of the stencil;
            if (flagsBuffer[indexCtr] == INDEX_ACTIVE_FLAG) {
                auto indexPlusX  = acc.getValue(nanovdb::Coord(coord.x()+1, coord.y()  , coord.z()  ));
                auto indexMinusX = acc.getValue(nanovdb::Coord(coord.x()-1, coord.y()  , coord.z()  ));
                auto indexPlusY  = acc.getValue(nanovdb::Coord(coord.x()  , coord.y()+1, coord.z()  ));
                auto indexMinusY = acc.getValue(nanovdb::Coord(coord.x()  , coord.y()-1, coord.z()  ));
                auto indexPlusZ  = acc.getValue(nanovdb::Coord(coord.x()  , coord.y()  , coord.z()+1));
                auto indexMinusZ = acc.getValue(nanovdb::Coord(coord.x()  , coord.y()  , coord.z()-1));
                LuBuffer[indexCtr] =
                    -6 * uBuffer[indexCtr]
                    + uBuffer[indexPlusX]
                    + uBuffer[indexMinusX]
                    + uBuffer[indexPlusY]
                    + uBuffer[indexMinusY]
                    + uBuffer[indexPlusZ]
                    + uBuffer[indexMinusZ];
            }
        }
}


