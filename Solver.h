#pragma once

#include <vector>
#include "editor/mesh/MeshWorkerProtocol.h"

namespace Solver
{
    bool Solve(
        std::vector<AnalysisMeshNode>& nodes,
        std::vector<AnalysisMeshTriangle>& triangles,
        const std::vector<AnalysisMeshBoundaryEdge>& boundaryEdges,
        const std::vector<StructuralMaterial>& materials,
        const std::vector<Thickness>& thicknesses,
        const std::vector<EdgeDistributedLoad>& distributedLoads);
}
