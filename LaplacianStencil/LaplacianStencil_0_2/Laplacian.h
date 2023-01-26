#pragma once

#define XDIM 4096
#define YDIM 4096

void ComputeLaplacian(const float (&u)[XDIM][YDIM], float (&Lu)[XDIM][YDIM]);
