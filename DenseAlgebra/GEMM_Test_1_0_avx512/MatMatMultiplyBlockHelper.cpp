#include "MatMatMultiplyBlockHelper.h"
#include "immintrin.h"

void MatMatMultiplyBlockHelper(const float (&bA)[BLOCK_SIZE][BLOCK_SIZE],
    const float (&bB)[BLOCK_SIZE][BLOCK_SIZE], float (&bC)[BLOCK_SIZE][BLOCK_SIZE])
{
    static constexpr int nW = 16; // Width of SIMD vectors

    for (int ii = 0; ii < BLOCK_SIZE; ii++)
    for (int kk = 0; kk < BLOCK_SIZE; kk++) 
        for (int jj = 0; jj < BLOCK_SIZE; jj += nW) {
            __m512 vB = _mm512_load_ps(&bB[kk][jj]);
            __m512 vA = _mm512_set1_ps(bA[ii][kk]);
            __m512 vC = _mm512_load_ps(&bC[ii][jj]);
            vC = _mm512_fmadd_ps(vA, vB, vC);
            _mm512_store_ps(&bC[ii][jj], vC);
        }
}
