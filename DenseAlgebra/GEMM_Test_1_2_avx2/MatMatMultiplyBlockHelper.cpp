#include "MatMatMultiplyBlockHelper.h"
#include "immintrin.h"

void MatMatMultiplyBlockHelper(const float (&bA)[BLOCK_SIZE][BLOCK_SIZE],
    const float (&bB)[BLOCK_SIZE][BLOCK_SIZE], float (&bC)[BLOCK_SIZE][BLOCK_SIZE])
{
    static constexpr int nW = 8;  // Width of a SIMD vector
    static constexpr int nB = BLOCK_SIZE/nW;
    using const_blocked_matrix_t = const float (&) [nB][nW][nB][nW];
    using blocked_matrix_t = float (&) [nB][nW][nB][nW];

    // Matrix bA has indices [i][k], or in block form (bbA) [bi][ii][bk][kk]
    // Matrix bB has indices [k][j], or in block form (bbB) [bk][kk][bj][jj]
    // matrix bC has indices [i][j], or in block form (bbC) [bi][ii][bj][jj]

    auto bbA = reinterpret_cast<const_blocked_matrix_t>(bA[0][0]);
    auto bbB = reinterpret_cast<const_blocked_matrix_t>(bB[0][0]);
    auto bbC = reinterpret_cast<blocked_matrix_t>(bC[0][0]);

    for (int bi = 0; bi < nB; bi++)
    for (int bj = 0; bj < nB; bj++)
    {
        __m256 vC[nW];
        for (int ii = 0; ii < nW; ii++)
            vC[ii] = _mm256_load_ps(&bbC[bi][ii][bj][0]);

        for (int bk = 0; bk < nB; bk++) {
            __m256 vB[nW];
            for (int kk = 0; kk < nW; kk++)
                vB[kk] = _mm256_load_ps(&bbB[bk][kk][bj][0]);

            for (int ii = 0; ii < nW; ii++)
            for (int kk = 0; kk < nW; kk++)
                vC[ii] = _mm256_fmadd_ps(_mm256_set1_ps(bbA[bi][ii][bk][kk]), vB[kk], vC[ii]);
        }

        for (int ii = 0; ii < nW; ii++)
            _mm256_store_ps(&bbC[bi][ii][bj][0],vC[ii]);
    }
}
