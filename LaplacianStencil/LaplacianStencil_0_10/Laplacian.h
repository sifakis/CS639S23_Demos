#pragma once

#define XDIM 512
#define YDIM 512
#define ZDIM 512

void ComputeLaplacian(const float (&u)[XDIM][YDIM][ZDIM], float (&Lu)[XDIM][YDIM][ZDIM]);
