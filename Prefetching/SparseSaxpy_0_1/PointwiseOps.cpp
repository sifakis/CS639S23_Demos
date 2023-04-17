#include "PointwiseOps.h"
#include "immintrin.h"

void SparseSaxpy(std::vector<int>& blockOffsets, const float *x, float *y, const float scale)
{
    static constexpr int L2_PREFETCH_DISTANCE = 64;
    static constexpr int L1_PREFETCH_DISTANCE = 8;
    
#pragma omp parallel for
    for (int b = 0; b < blockOffsets.size(); b++) {
        _mm_prefetch ( reinterpret_cast<const char *>(&x[blockOffsets[b+L2_PREFETCH_DISTANCE]]), _MM_HINT_T2 );
        _mm_prefetch ( reinterpret_cast<const char *>(&x[blockOffsets[b+L1_PREFETCH_DISTANCE]]), _MM_HINT_T1 );
        _mm_prefetch ( reinterpret_cast<const char *>(&y[blockOffsets[b+L2_PREFETCH_DISTANCE]]), _MM_HINT_T2 );
        _mm_prefetch ( reinterpret_cast<const char *>(&y[blockOffsets[b+L1_PREFETCH_DISTANCE]]), _MM_HINT_T1 );
#pragma omp simd
        for (int i = 0; i < BLOCK_SIZE; i++)
            y[blockOffsets[b]+i] += scale * x[blockOffsets[b]+i];
    }
}

