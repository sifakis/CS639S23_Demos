#pragma once

#include <memory>

struct CSRMatrix
{
    int mSize;
    std::unique_ptr<int> mRowOffsets;
    std::unique_ptr<int> mColumnIndices;
    std::unique_ptr<float> mValues;

    int* GetRowOffsets() { return mRowOffsets.get(); }
    int* GetColumnIndices() { return mColumnIndices.get(); }
    float* GetValues() { return mValues.get(); }

    void CheckCSRMatrix() {
        const auto rowOffsets = GetRowOffsets();
        const auto columnIndices = GetColumnIndices();
        const auto values = GetValues();

        if (rowOffsets[0] != 0)
            throw std::logic_error("In CSRMatrix::CheckCSRMatrix() -- nonzero starting row offset detected");
        for (int i = 0; i < mSize; i++) {
            if (rowOffsets[i+1] <= rowOffsets[i])
                throw std::logic_error("In CSRMatrix::CheckCSRMatrix() -- blank or invalid length sparse row detected");
            for (int k = rowOffsets[i]; k < rowOffsets[i+1]; k++) {
                if (columnIndices[k] < 0 || columnIndices[k] >= mSize)
                    throw std::logic_error("In CSRMatrix::CheckCSRMatrix() -- invalid column index");
                if (k > rowOffsets[i] && columnIndices[k] <= columnIndices[k-1])
                    throw std::logic_error("In CSRMatrix::CheckCSRMatrix() -- sparse row is not sorted");
            }
        }        
    }

    void CheckLowerTriangular() {
        const auto rowOffsets = GetRowOffsets();
        const auto columnIndices = GetColumnIndices();
        const auto values = GetValues();

        if (rowOffsets[0] != 0)
            throw std::logic_error("In CSRMatrix::CheckLowerTriangular() -- nonzero starting row offset detected");
        for (int i = 0; i < mSize; i++) {
            if (rowOffsets[i+1] <= rowOffsets[i])
                throw std::logic_error("In CSRMatrix::CheckLowerTriangular() -- blank or invalid length sparse row detected");
            for (int k = rowOffsets[i]; k < rowOffsets[i+1]; k++) {
                if (columnIndices[k] > i || columnIndices[k] >= mSize)
                    throw std::logic_error("In CSRMatrix::CheckLowerTriangular() -- invalid column index");
                if (k > rowOffsets[i] && columnIndices[k] <= columnIndices[k-1])
                    throw std::logic_error("In CSRMatrix::CheckLowerTriangular() -- sparse row is not sorted");
                if (k == rowOffsets[i+1]-1 && columnIndices[k] != i)
                    throw std::logic_error("In CSRMatrix::CheckLowerTriangular() -- diagonal element missing");
            }
        }        
    }

    void CheckUpperTriangular() {
        const auto rowOffsets = GetRowOffsets();
        const auto columnIndices = GetColumnIndices();
        const auto values = GetValues();

        if (rowOffsets[0] != 0)
            throw std::logic_error("In CSRMatrix::CheckUpperTriangular() -- nonzero starting row offset detected");
        for (int i = 0; i < mSize; i++) {
            if (rowOffsets[i+1] <= rowOffsets[i])
                throw std::logic_error("In CSRMatrix::CheckUpperTriangular() -- blank or invalid length sparse row detected");
            for (int k = rowOffsets[i]; k < rowOffsets[i+1]; k++) {
                if (columnIndices[k] < i || columnIndices[k] >= mSize)
                    throw std::logic_error("In CSRMatrix::CheckUpperTriangular() -- invalid column index");
                if (k > rowOffsets[i] && columnIndices[k] <= columnIndices[k-1])
                    throw std::logic_error("In CSRMatrix::CheckUpperTriangular() -- sparse row is not sorted");
                if (k == rowOffsets[i] && columnIndices[k] != i)
                    throw std::logic_error("In CSRMatrix::CheckUpperTriangular() -- diagonal element missing");
            }
        }
        
    }
};
