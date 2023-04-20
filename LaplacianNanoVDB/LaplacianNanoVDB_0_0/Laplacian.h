#pragma once

#include <nanovdb/NanoVDB.h>
#include <nanovdb/util/GridHandle.h>

#define XDIM 512
#define YDIM 512
#define ZDIM 512
#define INDEX_ACTIVE_FLAG 0xffffffff

void ComputeLaplacian(const float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM]);

nanovdb::GridHandle<nanovdb::HostBuffer> initializeIndexGrid();

void initializeData(float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM],
    nanovdb::NanoGrid<nanovdb::ValueIndex>* indexGridPtr,
    float *uBuffer, float *LuBuffer, uint32_t* flagsBuffer);

float compareData(const float (&data)[XDIM][YDIM][ZDIM],
    nanovdb::NanoGrid<nanovdb::ValueIndex>* indexGridPtr,
    const float *dataVDBBuffer);

void computeLaplacianVDB(nanovdb::NanoGrid<nanovdb::ValueIndex>* indexGridPtr,
    float *uBuffer, float *LuBuffer, uint32_t* flagsBuffer);
