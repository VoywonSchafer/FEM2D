#include "Solver.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/SparseCholesky>
#include <iostream>
#include <fstream>
#include <cmath>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <exception>

namespace Solver
{
    namespace
    {
        void Log(const std::string& msg)
        {
            std::ofstream debugLog("worker_debug.log", std::ios::out | std::ios::app);
            debugLog << "[SOLVER] " << msg << std::endl;
        }

        double Distance(const Point2D& a, const Point2D& b)
        {
            return std::sqrt(std::pow(b.x - a.x, 2) + std::pow(b.y - a.y, 2));
        }
    }

    struct Edge
    {
        int v1, v2;
        bool operator<(const Edge& other) const
        {
            if (v1 != other.v1) return v1 < other.v1;
            return v2 < other.v2;
        }
    };

    Edge MakeEdge(int a, int b)
    {
        return { std::min(a, b), std::max(a, b) };
    }

    bool Solve(
        std::vector<AnalysisMeshNode>& nodes,
        std::vector<AnalysisMeshTriangle>& triangles,
        const std::vector<AnalysisMeshBoundaryEdge>& boundaryEdges,
        const std::vector<StructuralMaterial>& materials,
        const std::vector<Thickness>& thicknesses,
        const std::vector<EdgeDistributedLoad>& distributedLoads)
    {
        try
        {
            Log("Iniciando Solve (LST + Load Integration Fix)...");
            if (nodes.empty() || triangles.empty())
            {
                Log("ERRO: Nodes ou Triangles vazios.");
                return false;
            }

            std::unordered_map<int, const StructuralMaterial*> materialMap;
            for (const auto& mat : materials) materialMap[mat.id] = &mat;

            std::unordered_map<int, const Thickness*> thicknessMap;
            for (const auto& thk : thicknesses) thicknessMap[thk.id] = &thk;

            std::unordered_map<int, const EdgeDistributedLoad*> loadMap;
            for (const auto& load : distributedLoads) loadMap[load.edgeId] = &load;

            Log("1. Construindo Mid-Nodes (LST)...");
            std::map<Edge, int> edgeToMidNode;
            std::vector<AnalysisMeshNode> corners = nodes; 
            std::vector<AnalysisMeshNode> allNodes = corners;

            for (const auto& tri : triangles)
            {
                int i = tri.nodeIndices[0];
                int j = tri.nodeIndices[1];
                int k = tri.nodeIndices[2];

                Edge edges[3] = { MakeEdge(i, j), MakeEdge(j, k), MakeEdge(k, i) };
                for (int e = 0; e < 3; ++e)
                {
                    if (edgeToMidNode.find(edges[e]) == edgeToMidNode.end())
                    {
                        int n1 = edges[e].v1;
                        int n2 = edges[e].v2;
                        AnalysisMeshNode mid;
                        mid.position.x = (corners[n1].position.x + corners[n2].position.x) / 2.0;
                        mid.position.y = (corners[n1].position.y + corners[n2].position.y) / 2.0;
                        mid.fx = 0.0;
                        mid.fy = 0.0;
                        mid.restraintX = false;
                        mid.restraintY = false;
                        
                        int index = static_cast<int>(allNodes.size());
                        edgeToMidNode[edges[e]] = index;
                        allNodes.push_back(mid);
                    }
                }
            }

            const size_t numNodes = allNodes.size();
            const size_t numDofs = numNodes * 2;
            Log("   - Total de nos (com mid-nodes): " + std::to_string(numNodes));

            Log("2. Montagem da Matriz de Rigidez K...");
            std::vector<Eigen::Triplet<double>> triplets;
            triplets.reserve(triangles.size() * 144 + boundaryEdges.size() * 6);

            Eigen::VectorXd F = Eigen::VectorXd::Zero(numDofs);

            double totalForceY = 0.0;
            int loadedEdgesCount = 0;

            // Integrate loads
            for (const auto& bEdge : boundaryEdges)
            {
                const auto loadIt = loadMap.find(bEdge.cadEdgeId);
                int n1 = bEdge.nodeIndex0;
                int n2 = bEdge.nodeIndex1;
                auto itMid = edgeToMidNode.find(MakeEdge(n1, n2));
                if (itMid == edgeToMidNode.end()) continue;
                int m12 = itMid->second;

                double L = Distance(allNodes[n1].position, allNodes[n2].position);

                if (loadIt != loadMap.end())
                {
                    const auto& loadValue = loadIt->second->value;
                    // LST nodal forces for uniform load: 1/6 for corners, 4/6 for mid-node
                    // We assume uniform load for simplicity or average the start/end if they differ
                    double qx = (loadValue.qxStart + loadValue.qxEnd) / 2.0;
                    double qy = (loadValue.qyStart + loadValue.qyEnd) / 2.0;

                    F(n1 * 2)     += (qx * L) / 6.0;
                    F(n1 * 2 + 1) += (qy * L) / 6.0;

                    F(n2 * 2)     += (qx * L) / 6.0;
                    F(n2 * 2 + 1) += (qy * L) / 6.0;

                    F(m12 * 2)     += (2.0 * qx * L) / 3.0;
                    F(m12 * 2 + 1) += (2.0 * qy * L) / 3.0;

                    totalForceY += qy * L;
                    loadedEdgesCount++;
                }
            }
            Log("   - Arestas com carga encontradas: " + std::to_string(loadedEdgesCount));
            Log("   - Forca total aplicada em Y: " + std::to_string(totalForceY));

            // Also add point loads from the original corner nodes
            for (size_t i = 0; i < nodes.size(); ++i)
            {
                F(i * 2) += allNodes[i].fx;
                F(i * 2 + 1) += allNodes[i].fy;
            }

            for (auto& tri : triangles)
            {
                const auto matIt = materialMap.find(tri.materialId);
                const auto thkIt = thicknessMap.find(tri.thicknessId);
                
                double E_loc = -1.0, nu_loc = 0.0, t_loc = -1.0;
                if (matIt != materialMap.end()) { E_loc = matIt->second->youngModulus; nu_loc = matIt->second->poissonRatio; }
                else if (!materials.empty()) { E_loc = materials[0].youngModulus; nu_loc = materials[0].poissonRatio; }
                if (thkIt != thicknessMap.end()) { t_loc = thkIt->second->thickness; }
                else if (!thicknesses.empty()) { t_loc = thicknesses[0].thickness; }

                int n1 = tri.nodeIndices[0], n2 = tri.nodeIndices[1], n3 = tri.nodeIndices[2];
                int m12 = edgeToMidNode[MakeEdge(n1, n2)];
                int m23 = edgeToMidNode[MakeEdge(n2, n3)];
                int m31 = edgeToMidNode[MakeEdge(n3, n1)];

                double x1 = allNodes[n1].position.x, y1 = allNodes[n1].position.y;
                double x2 = allNodes[n2].position.x, y2 = allNodes[n2].position.y;
                double x3 = allNodes[n3].position.x, y3 = allNodes[n3].position.y;

                double area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
                double absArea = std::abs(area);

                if (absArea < 1e-15 || E_loc <= 0.0 || t_loc <= 0.0) continue;

                Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
                D << 1.0, nu_loc, 0.0, nu_loc, 1.0, 0.0, 0.0, 0.0, (1.0 - nu_loc) / 2.0;
                D *= (E_loc / (1.0 - nu_loc * nu_loc));

                Eigen::Matrix<double, 12, 12> Ke = Eigen::Matrix<double, 12, 12>::Zero();
                double gaussPoints[3][3] = { {0.5, 0.5, 0.0}, {0.0, 0.5, 0.5}, {0.5, 0.0, 0.5} };
                double weight = 1.0 / 3.0;

                double b1 = y2 - y3, b2 = y3 - y1, b3 = y1 - y2;
                double a1 = x3 - x2, a2 = x1 - x3, a3 = x2 - x1;
                double inv2A = 1.0 / (2.0 * area);

                for (int p = 0; p < 3; ++p)
                {
                    double xi1 = gaussPoints[p][0], xi2 = gaussPoints[p][1], xi3 = gaussPoints[p][2];
                    Eigen::Matrix<double, 2, 6> dN_dxi;
                    dN_dxi(0, 0) = 4.0 * xi1 - 1.0; dN_dxi(1, 0) = 0.0;
                    dN_dxi(0, 1) = 0.0;             dN_dxi(1, 1) = 4.0 * xi2 - 1.0;
                    dN_dxi(0, 2) = -(4.0 * xi3 - 1.0); dN_dxi(1, 2) = -(4.0 * xi3 - 1.0);
                    dN_dxi(0, 3) = 4.0 * xi2;       dN_dxi(1, 3) = 4.0 * xi1;
                    dN_dxi(0, 4) = -4.0 * xi2;      dN_dxi(1, 4) = 4.0 * (xi3 - xi2);
                    dN_dxi(0, 5) = 4.0 * (xi3 - xi1); dN_dxi(1, 5) = -4.0 * xi1;

                    Eigen::Matrix<double, 2, 6> dN_dx;
                    for (int i = 0; i < 6; ++i) {
                        dN_dx(0, i) = inv2A * (dN_dxi(0, i) * b1 + dN_dxi(1, i) * b2);
                        dN_dx(1, i) = inv2A * (dN_dxi(0, i) * a1 + dN_dxi(1, i) * a2);
                    }
                    Eigen::Matrix<double, 3, 12> B = Eigen::Matrix<double, 3, 12>::Zero();
                    for (int i = 0; i < 6; ++i) {
                        B(0, 2 * i) = dN_dx(0, i); B(1, 2 * i + 1) = dN_dx(1, i);
                        B(2, 2 * i) = dN_dx(1, i); B(2, 2 * i + 1) = dN_dx(0, i);
                    }
                    Ke += B.transpose() * D * B * weight * t_loc * absArea;
                }

                int localToGlobal[6] = { n1, n2, n3, m12, m23, m31 };
                tri.midNodeIndices = { m12, m23, m31 };
                for (int i = 0; i < 6; ++i) {
                    int gi = localToGlobal[i];
                    for (int j = 0; j < 6; ++j) {
                        int gj = localToGlobal[j];
                        triplets.push_back(Eigen::Triplet<double>(gi * 2, gj * 2, Ke(i * 2, j * 2)));
                        triplets.push_back(Eigen::Triplet<double>(gi * 2, gj * 2 + 1, Ke(i * 2, j * 2 + 1)));
                        triplets.push_back(Eigen::Triplet<double>(gi * 2 + 1, gj * 2, Ke(i * 2 + 1, j * 2)));
                        triplets.push_back(Eigen::Triplet<double>(gi * 2 + 1, gj * 2 + 1, Ke(i * 2 + 1, j * 2 + 1)));
                    }
                }
            }

            Log("3. Aplicando Condicoes de Contorno...");
            const double penalty = 1e15;
            for (size_t i = 0; i < nodes.size(); ++i) {
                if (allNodes[i].restraintX) { triplets.push_back(Eigen::Triplet<double>(static_cast<int>(i * 2), static_cast<int>(i * 2), penalty)); F(i * 2) = 0.0; }
                if (allNodes[i].restraintY) { triplets.push_back(Eigen::Triplet<double>(static_cast<int>(i * 2 + 1), static_cast<int>(i * 2 + 1), penalty)); F(i * 2 + 1) = 0.0; }
            }
            for (const auto& bEdge : boundaryEdges) {
                int n1 = bEdge.nodeIndex0; int n2 = bEdge.nodeIndex1;
                auto itMid = edgeToMidNode.find(MakeEdge(n1, n2));
                if (itMid == edgeToMidNode.end()) continue;
                int m12 = itMid->second;
                if (allNodes[n1].restraintX && allNodes[n2].restraintX) { triplets.push_back(Eigen::Triplet<double>(m12 * 2, m12 * 2, penalty)); F(m12 * 2) = 0.0; }
                if (allNodes[n1].restraintY && allNodes[n2].restraintY) { triplets.push_back(Eigen::Triplet<double>(m12 * 2 + 1, m12 * 2 + 1, penalty)); F(m12 * 2 + 1) = 0.0; }
            }

            Log("4. Resolvendo sistema linear...");
            Eigen::SparseMatrix<double> K(numDofs, numDofs);
            K.setFromTriplets(triplets.begin(), triplets.end());
            K.makeCompressed();

            Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
            solver.compute(K);
            if (solver.info() != Eigen::Success) { Log("ERRO: Eigen falhou no compute."); return false; }
            Eigen::VectorXd U = solver.solve(F);
            if (solver.info() != Eigen::Success) { Log("ERRO: Eigen falhou no solve."); return false; }

            Log("5. Calculando tensoes...");
            std::vector<AnalysisMeshNode> finalNodes = allNodes;
            for (size_t i = 0; i < finalNodes.size(); ++i) {
                finalNodes[i].displacement.x = U(i * 2);
                finalNodes[i].displacement.y = U(i * 2 + 1);
            }

            struct NodalStressSum { double sx = 0, sy = 0, txy = 0, count = 0; };
            std::vector<NodalStressSum> nodalStressSums(finalNodes.size());

            for (auto& tri : triangles)
            {
                const auto matIt = materialMap.find(tri.materialId);
                const auto thkIt = thicknessMap.find(tri.thicknessId);
                double E_loc = -1.0, nu_loc = 0.0, t_loc = -1.0;
                if (matIt != materialMap.end()) { E_loc = matIt->second->youngModulus; nu_loc = matIt->second->poissonRatio; }
                else if (!materials.empty()) { E_loc = materials[0].youngModulus; nu_loc = materials[0].poissonRatio; }
                if (thkIt != thicknessMap.end()) { t_loc = thkIt->second->thickness; }
                else if (!thicknesses.empty()) { t_loc = thicknesses[0].thickness; }
                if (E_loc <= 0.0) continue;

                int n1 = tri.nodeIndices[0], n2 = tri.nodeIndices[1], n3 = tri.nodeIndices[2];
                int m12 = edgeToMidNode[MakeEdge(n1, n2)];
                int m23 = edgeToMidNode[MakeEdge(n2, n3)];
                int m31 = edgeToMidNode[MakeEdge(n3, n1)];

                double x1 = finalNodes[n1].position.x, y1 = finalNodes[n1].position.y;
                double x2 = finalNodes[n2].position.x, y2 = finalNodes[n2].position.y;
                double x3 = finalNodes[n3].position.x, y3 = finalNodes[n3].position.y;
                double area = 0.5 * (x1 * (y2 - y3) + x2 * (y3 - y1) + x3 * (y1 - y2));
                if (std::abs(area) < 1e-15) continue;

                Eigen::Matrix3d D = Eigen::Matrix3d::Zero();
                D << 1.0, nu_loc, 0.0, nu_loc, 1.0, 0.0, 0.0, 0.0, (1.0 - nu_loc) / 2.0;
                D *= (E_loc / (1.0 - nu_loc * nu_loc));

                Eigen::Matrix<double, 12, 1> u_elem;
                int gdl[12] = { n1*2, n1*2+1, n2*2, n2*2+1, n3*2, n3*2+1, m12*2, m12*2+1, m23*2, m23*2+1, m31*2, m31*2+1 };
                for (int i = 0; i < 12; ++i) u_elem(i) = U(gdl[i]);

                double sumSx = 0, sumSy = 0, sumTxy = 0;
                auto eval = [&](double xi1, double xi2, double xi3, int nodeIdx) {
                    Eigen::Matrix<double, 2, 6> dN_dxi;
                    dN_dxi(0, 0) = 4.0 * xi1 - 1.0; dN_dxi(1, 0) = 0.0;
                    dN_dxi(0, 1) = 0.0;             dN_dxi(1, 1) = 4.0 * xi2 - 1.0;
                    dN_dxi(0, 2) = -(4.0 * xi3 - 1.0); dN_dxi(1, 2) = -(4.0 * xi3 - 1.0);
                    dN_dxi(0, 3) = 4.0 * xi2;       dN_dxi(1, 3) = 4.0 * xi1;
                    dN_dxi(0, 4) = -4.0 * xi2;      dN_dxi(1, 4) = 4.0 * (xi3 - xi2);
                    dN_dxi(0, 5) = 4.0 * (xi3 - xi1); dN_dxi(1, 5) = -4.0 * xi1;

                    double inv2A = 1.0 / (2.0 * area);
                    Eigen::Matrix<double, 2, 6> dN_dx;
                    for (int i = 0; i < 6; ++i) {
                        dN_dx(0, i) = inv2A * (dN_dxi(0, i) * (y2 - y3) + dN_dxi(1, i) * (y3 - y1));
                        dN_dx(1, i) = inv2A * (dN_dxi(0, i) * (x3 - x2) + dN_dxi(1, i) * (x1 - x3));
                    }
                    Eigen::Matrix<double, 3, 12> B = Eigen::Matrix<double, 3, 12>::Zero();
                    for (int i = 0; i < 6; ++i) {
                        B(0, 2 * i) = dN_dx(0, i); B(1, 2 * i + 1) = dN_dx(1, i);
                        B(2, 2 * i) = dN_dx(1, i); B(2, 2 * i + 1) = dN_dx(0, i);
                    }
                    Eigen::Vector3d stress = D * B * u_elem;
                    if (nodeIdx >= 0) {
                        nodalStressSums[nodeIdx].sx += stress(0); nodalStressSums[nodeIdx].sy += stress(1);
                        nodalStressSums[nodeIdx].txy += stress(2); nodalStressSums[nodeIdx].count += 1.0;
                    }
                    sumSx += stress(0); sumSy += stress(1); sumTxy += stress(2);
                };

                eval(1.0, 0.0, 0.0, n1); eval(0.0, 1.0, 0.0, n2); eval(0.0, 0.0, 1.0, n3);
                eval(0.5, 0.5, 0.0, m12); eval(0.0, 0.5, 0.5, m23); eval(0.5, 0.0, 0.5, m31);

                tri.stressX = sumSx / 6.0; tri.stressY = sumSy / 6.0; tri.shearStress = sumTxy / 6.0;
                tri.vonMises = std::sqrt(tri.stressX * tri.stressX - tri.stressX * tri.stressY + tri.stressY * tri.stressY + 3.0 * tri.shearStress * tri.shearStress);
            }

            for (size_t i = 0; i < finalNodes.size(); ++i) {
                if (nodalStressSums[i].count > 0) {
                    finalNodes[i].stressX = nodalStressSums[i].sx / nodalStressSums[i].count;
                    finalNodes[i].stressY = nodalStressSums[i].sy / nodalStressSums[i].count;
                    finalNodes[i].shearStress = nodalStressSums[i].txy / nodalStressSums[i].count;
                    finalNodes[i].vonMises = std::sqrt(finalNodes[i].stressX * finalNodes[i].stressX - finalNodes[i].stressX * finalNodes[i].stressY + finalNodes[i].stressY * finalNodes[i].stressY + 3.0 * finalNodes[i].shearStress * finalNodes[i].shearStress);
                }
            }

            nodes = std::move(finalNodes);

            double maxDisp = 0.0;
            for (const auto& n : nodes) {
                maxDisp = std::max(maxDisp, std::sqrt(n.displacement.x * n.displacement.x + n.displacement.y * n.displacement.y));
            }
            double maxVM = 0.0;
            for (const auto& tri : triangles) {
                maxVM = std::max(maxVM, tri.vonMises);
            }
            Log("   - Deslocamento maximo: " + std::to_string(maxDisp));
            Log("   - Tensao de Von Mises maxima: " + std::to_string(maxVM));

            Log("Solve concluido com sucesso.");
            return true;
        }
        catch (const std::exception& e) { Log("CRASH FATAL: " + std::string(e.what())); return false; }
        catch (...) { Log("CRASH FATAL desconhecido."); return false; }
    }
}
