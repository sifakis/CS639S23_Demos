#pragma once

#define XDIM 16384
#define YDIM 16384

void ComputeLaplacian(const float (&u)[XDIM][YDIM], float (&Lu)[XDIM][YDIM]);
