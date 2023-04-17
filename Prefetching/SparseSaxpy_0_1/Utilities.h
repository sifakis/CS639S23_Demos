#pragma once

#include <vector>

#include "Parameters.h"

void* AlignedAllocate(const std::size_t size, const std::size_t alignment);
void InitializeArrays(std::vector<int>& blockOffsets, float *&x, float *&y);
