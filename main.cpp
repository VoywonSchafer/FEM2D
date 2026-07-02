#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>
#include <fstream>

#if defined(PLATFORM_WEB)
#include <emscripten/emscripten.h>
#else
#include <fcntl.h>
#include <io.h>
#endif

#include "editor/mesh/MeshGenerationService.h"
#include "editor/mesh/MeshWorkerProtocol.h"
#include "Solver.h"

namespace
{
#if !defined(PLATFORM_WEB)
    std::ofstream debugLog("worker_debug.log", std::ios::out | std::ios::app);
#endif

#if defined(PLATFORM_WEB)
    void PostErrorToParent(const char* message)
    {
        EM_ASM({
            postMessage({ type: "error", message: UTF8ToString($0) });
        }, message);
    }
#endif

    bool ReadExactFrame(
        std::istream& input,
        MeshWorkerProtocol::MessageType& type,
        std::vector<std::uint8_t>& payload)
    {
        MeshWorkerProtocol::FrameHeader header;
        input.read(reinterpret_cast<char*>(&header), sizeof(header));
        if (input.gcount() == 0)
        {
            return false;
        }
        if (input.gcount() != sizeof(header))
        {
            return false;
        }
        if (header.magic != MeshWorkerProtocol::FrameMagic ||
            header.version != MeshWorkerProtocol::ProtocolVersion)
        {
            return false;
        }

        payload.resize(header.payloadSize);
        if (header.payloadSize > 0)
        {
            input.read(reinterpret_cast<char*>(payload.data()), static_cast<std::streamsize>(payload.size()));
            if (input.gcount() != static_cast<std::streamsize>(payload.size()))
            {
                return false;
            }
        }

        type = static_cast<MeshWorkerProtocol::MessageType>(header.type);
        return true;
    }

    bool WriteFrame(
        std::ostream& output,
        MeshWorkerProtocol::MessageType type,
        const std::vector<std::uint8_t>& payload)
    {
        const std::vector<std::uint8_t> frame = MeshWorkerProtocol::BuildFrame(type, payload);
        output.write(reinterpret_cast<const char*>(frame.data()), static_cast<std::streamsize>(frame.size()));
        output.flush();
        
#if !defined(PLATFORM_WEB)
        // Explicitly flush stdout at the OS level to ensure the editor receives the data
        _commit(_fileno(stdout));
#endif
        return output.good();
    }

#if defined(PLATFORM_WEB)
    bool PostFrameToParent(
        MeshWorkerProtocol::MessageType type,
        const std::vector<std::uint8_t>& payload)
    {
        const std::vector<std::uint8_t> frame = MeshWorkerProtocol::BuildFrame(type, payload);
        EM_ASM({
            const bytes = HEAPU8.slice($0, $0 + $1);
            postMessage({ type: "frame", bytes: bytes.buffer }, [bytes.buffer]);
        }, frame.data(), static_cast<int>(frame.size()));
        return true;
    }

    int ExecuteJobFromPayload(const std::vector<std::uint8_t>& requestPayload)
    {
        ProjectDocument document;
        int smoothingIterations = 0;
        if (!MeshWorkerProtocol::ParseJobRequestPayload(requestPayload, document, smoothingIterations))
        {
            PostErrorToParent("Failed to parse mesh job payload.");
            return 2;
        }

        ProjectMeshState meshState;
        MeshGenerationService::SyncMeshStateWithDocument(meshState, document);

        std::function<void(int, int, int)> onAreaBegin =
            [](int areaId, int areaIndex, int totalAreas)
            {
                MeshWorkerProtocol::AreaStartedMessage message;
                message.areaId = areaId;
                message.areaIndex = areaIndex;
                message.totalAreas = totalAreas;
                PostFrameToParent(
                    MeshWorkerProtocol::MessageType::AreaStarted,
                    MeshWorkerProtocol::BuildAreaStartedPayload(message));
            };

        std::function<void(int, int, int)> onAreaCompleted =
            [&meshState](int areaId, int completedAreas, int totalAreas)
            {
                const auto meshIt = meshState.areaMeshesById.find(areaId);
                if (meshIt == meshState.areaMeshesById.end())
                {
                    return;
                }

                meshIt->second.status = AreaMeshStatus::Ready;
                meshIt->second.hasFailureHighlight = false;

                MeshWorkerProtocol::AreaMeshReadyMessage message;
                message.areaId = areaId;
                message.completedAreas = completedAreas;
                message.totalAreas = totalAreas;
                message.mesh = meshIt->second;
                PostFrameToParent(
                    MeshWorkerProtocol::MessageType::AreaMeshReady,
                    MeshWorkerProtocol::BuildAreaMeshReadyPayload(message));
            };

        const bool success = MeshGenerationService::GenerateAllAreaMeshes(
            meshState,
            document,
            smoothingIterations,
            &onAreaBegin,
            &onAreaCompleted,
            nullptr);

        if (!success)
        {
            MeshWorkerProtocol::JobFailedMessage message;
            message.failedAreaIds.assign(meshState.failedAreaIds.begin(), meshState.failedAreaIds.end());
            PostFrameToParent(
                MeshWorkerProtocol::MessageType::JobFailed,
                MeshWorkerProtocol::BuildJobFailedPayload(message));
            return 3;
        }

        MeshWorkerProtocol::JobSucceededMessage message;
        message.analysisNodes = meshState.analysisNodes;
        message.analysisTriangles = meshState.analysisTriangles;
        message.analysisBoundaryEdges = meshState.analysisBoundaryEdges;
        PostFrameToParent(
            MeshWorkerProtocol::MessageType::JobSucceeded,
            MeshWorkerProtocol::BuildJobSucceededPayload(message));
        return 0;
    }

    int ExecuteAnalysisJobFromPayload(const std::vector<std::uint8_t>& requestPayload)
    {
        MeshWorkerProtocol::AnalysisJobRequestMessage request;
        if (!MeshWorkerProtocol::ParseAnalysisRequestPayload(requestPayload, request))
        {
            PostErrorToParent("Failed to parse analysis job payload.");
            return 2;
        }

        if (!Solver::Solve(request.nodes, request.triangles, request.boundaryEdges, request.materials, request.thicknesses, request.distributedLoads))
        {
            PostFrameToParent(
                MeshWorkerProtocol::MessageType::AnalysisFailed,
                {});
            return 3;
        }

        MeshWorkerProtocol::JobSucceededMessage message;
        message.analysisNodes = std::move(request.nodes);
        message.analysisTriangles = std::move(request.triangles);
        PostFrameToParent(
            MeshWorkerProtocol::MessageType::AnalysisSucceeded,
            MeshWorkerProtocol::BuildAnalysisSucceededPayload(message));
        return 0;
    }

    extern "C"
    {
        EMSCRIPTEN_KEEPALIVE void MeshWorker_HandleFrame(const std::uint8_t* bytes, int size)
        {
            if (bytes == nullptr || size <= 0)
            {
                return;
            }

            std::vector<std::uint8_t> frame(bytes, bytes + size);
            MeshWorkerProtocol::MessageType requestType{};
            std::vector<std::uint8_t> requestPayload;
            if (!MeshWorkerProtocol::TryConsumeFrame(frame, requestType, requestPayload))
            {
                PostErrorToParent("Invalid job frame received.");
                EM_ASM({ postMessage({ type: "done" }); close(); });
                return;
            }

            if (requestType == MeshWorkerProtocol::MessageType::JobRequest)
            {
                ExecuteJobFromPayload(requestPayload);
            }
            else if (requestType == MeshWorkerProtocol::MessageType::AnalysisRequest)
            {
                ExecuteAnalysisJobFromPayload(requestPayload);
            }
            else
            {
                PostErrorToParent("Unknown job type received.");
            }

            EM_ASM({ postMessage({ type: "done" }); close(); });
        }
    }
#endif
}

int main()
{
#if defined(PLATFORM_WEB)
    emscripten_exit_with_live_runtime();
    return 0;
#else
    _setmode(_fileno(stdin), _O_BINARY);
    _setmode(_fileno(stdout), _O_BINARY);

    MeshWorkerProtocol::MessageType requestType{};
    std::vector<std::uint8_t> requestPayload;
    if (!ReadExactFrame(std::cin, requestType, requestPayload))
    {
        return 1;
    }

    if (requestType == MeshWorkerProtocol::MessageType::JobRequest)
    {
        debugLog << "1. JobRequest Recebido" << std::endl;
        ProjectDocument document;
        int smoothingIterations = 0;
        if (!MeshWorkerProtocol::ParseJobRequestPayload(requestPayload, document, smoothingIterations))
        {
            debugLog << "ERRO: Falha ao parsear JobRequest" << std::endl;
            return 2;
        }

        debugLog << "2. Payload lido com sucesso (JobRequest)" << std::endl;
        ProjectMeshState meshState;
        MeshGenerationService::SyncMeshStateWithDocument(meshState, document);

        std::function<void(int, int, int)> onAreaBegin =
            [](int areaId, int areaIndex, int totalAreas)
            {
                MeshWorkerProtocol::AreaStartedMessage message;
                message.areaId = areaId;
                message.areaIndex = areaIndex;
                message.totalAreas = totalAreas;
                WriteFrame(
                    std::cout,
                    MeshWorkerProtocol::MessageType::AreaStarted,
                    MeshWorkerProtocol::BuildAreaStartedPayload(message));
            };

        std::function<void(int, int, int)> onAreaCompleted =
            [&meshState](int areaId, int completedAreas, int totalAreas)
            {
                const auto meshIt = meshState.areaMeshesById.find(areaId);
                if (meshIt == meshState.areaMeshesById.end())
                {
                    return;
                }

                meshIt->second.status = AreaMeshStatus::Ready;
                meshIt->second.hasFailureHighlight = false;

                MeshWorkerProtocol::AreaMeshReadyMessage message;
                message.areaId = areaId;
                message.completedAreas = completedAreas;
                message.totalAreas = totalAreas;
                message.mesh = meshIt->second;

                WriteFrame(
                    std::cout,
                    MeshWorkerProtocol::MessageType::AreaMeshReady,
                    MeshWorkerProtocol::BuildAreaMeshReadyPayload(message));
            };

        const bool success = MeshGenerationService::GenerateAllAreaMeshes(
            meshState,
            document,
            smoothingIterations,
            &onAreaBegin,
            &onAreaCompleted,
            nullptr);

        if (!success)
        {
            debugLog << "ERRO: Falha na geracao de malha" << std::endl;
            MeshWorkerProtocol::JobFailedMessage message;
            message.failedAreaIds.assign(meshState.failedAreaIds.begin(), meshState.failedAreaIds.end());
            WriteFrame(
                std::cout,
                MeshWorkerProtocol::MessageType::JobFailed,
                MeshWorkerProtocol::BuildJobFailedPayload(message));
            return 3;
        }

        debugLog << "3. Malha gerada com sucesso" << std::endl;
        MeshWorkerProtocol::JobSucceededMessage message;
        message.analysisNodes = meshState.analysisNodes;
        message.analysisTriangles = meshState.analysisTriangles;
        message.analysisBoundaryEdges = meshState.analysisBoundaryEdges;
        WriteFrame(
            std::cout,
            MeshWorkerProtocol::MessageType::JobSucceeded,
            MeshWorkerProtocol::BuildJobSucceededPayload(message));
        debugLog << "4. Resposta JobSucceeded enviada" << std::endl;
        return 0;
    }
    else if (requestType == MeshWorkerProtocol::MessageType::AnalysisRequest)
    {
        debugLog << "--- NOVA ANALISE ---" << std::endl;
        debugLog << "1. AnalysisRequest Recebido" << std::endl;
        MeshWorkerProtocol::AnalysisJobRequestMessage request;
        if (!MeshWorkerProtocol::ParseAnalysisRequestPayload(requestPayload, request))
        {
            debugLog << "ERRO: Falha ao parsear AnalysisRequest" << std::endl;
            return 2;
        }

        debugLog << "2. Payload lido: " << request.nodes.size() << " nos, " << request.triangles.size() << " triangulos" << std::endl;
        debugLog << "3. Iniciando Solver::Solve..." << std::endl;
        
        bool success = Solver::Solve(request.nodes, request.triangles, request.boundaryEdges, request.materials, request.thicknesses, request.distributedLoads);
        
        if (!success)
        {
            debugLog << "ERRO: Solver falhou" << std::endl;
            WriteFrame(
                std::cout,
                MeshWorkerProtocol::MessageType::AnalysisFailed,
                {});
            return 3;
        }

        debugLog << "4. Sistema resolvido. Nos finais: " << request.nodes.size() << std::endl;
        MeshWorkerProtocol::JobSucceededMessage message;
        message.analysisNodes = std::move(request.nodes);
        message.analysisTriangles = std::move(request.triangles);
        
        debugLog << "5. Enviando resposta AnalysisSucceeded..." << std::endl;
        if (WriteFrame(
            std::cout,
            MeshWorkerProtocol::MessageType::AnalysisSucceeded,
            MeshWorkerProtocol::BuildAnalysisSucceededPayload(message)))
        {
            debugLog << "6. Resposta enviada com sucesso ao stdout" << std::endl;
        }
        else
        {
            debugLog << "ERRO: Falha ao escrever no stdout" << std::endl;
        }
        
        return 0;
    }

    return 4;
#endif
}
