#include "Utilities.h"

#include <cstring>
#include <memory>
#include <random>
#include <stdexcept>
#include <vector>

#include <iostream>

void* AlignedAllocate(const std::size_t size, const std::size_t alignment)
{
    std::size_t capacity = size + alignment - 1;
    void *ptr = new char[capacity];
    auto result = std::align(alignment, size, ptr, capacity);
    if (result == nullptr) throw std::bad_alloc();
    if (capacity < size) throw std::bad_alloc();
    return ptr;
}

void InitializeArrays(std::vector<int>& blockOffsets, float *&x, float *&y)
{
    std::random_device r; std::default_random_engine e(r());
    
    std::uniform_int_distribution<int> cluster_dist(1, MAX_CLUSTER_SIZE);
    std::uniform_int_distribution<int> distance_dist(1, MAX_CLUSTER_DISTANCE);    
    
    int currentOffset = 0;
    int nBlocks = 0;

    while (nBlocks < NUMBER_OF_BLOCKS)
    {
        // Add from 1 to MAX_CLUSTER_SIZE blocks
        int clusterSize = cluster_dist(e);
        if ( nBlocks + clusterSize > NUMBER_OF_BLOCKS )
            clusterSize = NUMBER_OF_BLOCKS - nBlocks;
        for (int b = 0; b < clusterSize; b++) {
            blockOffsets.push_back(currentOffset);
            currentOffset += BLOCK_SIZE;
            nBlocks++;
        }

        // If enough blocks, stop
        if (nBlocks == NUMBER_OF_BLOCKS) break;

        // Otherwise, create spacing of 1 to MAX_CLUSTER_DISTANCE
        int clusterDistance = distance_dist(e);
        currentOffset += BLOCK_SIZE * clusterDistance;
    }

    // Allocate arrays (ensuring alignment)
    x = static_cast<float*>(AlignedAllocate( currentOffset * sizeof(float), 64));
    y = static_cast<float*>(AlignedAllocate( currentOffset * sizeof(float), 64));
    std::memset(x, 0, currentOffset * sizeof(float));
    std::memset(y, 0, currentOffset * sizeof(float));

    std::cout << "Allocated total of " << nBlocks << " blocks ("
              << nBlocks * BLOCK_SIZE << " entries; "
              << (float)(nBlocks * BLOCK_SIZE * sizeof(float)) / 1048576.
              << "MB of actual data) in a span of "
              << (float)(currentOffset * sizeof(float) ) / 1048576. << "MB" << std::endl;
}
