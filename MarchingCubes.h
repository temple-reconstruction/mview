#ifndef MARCHING_CUBES_H
#define MARCHING_CUBES_H
#include "sdf.h"
#include "SimpleMesh.h"

bool ProcessVolumeCell(SdfIntegrator* vol, int x, int y, int z, double iso, SimpleMesh* mesh);

#endif
