#ifndef OSSIAN_CLIENT_GRAPHIC_WORKER
#define OSSIAN_CLIENT_GRAPHIC_WORKER

#include <ossian/Factory.hpp>
#include <ossian/IOListener.hpp>
#include <ossian/Pipeline.hpp>
#include <thread>
#include <chrono>
#include "ClientGraphic.hpp"

class ClientGraphicWorker : ossian::IExecutable {
    const std::unordered_map<uint16_t, std::shared_ptr<ClientGraphic>> &m_GraphicClients;

public:
    OSSIAN_SERVICE_SETUP(ClientGraphicWorker(ClientGraphicManager * manager))
            : m_GraphicClients(manager->GetGraphicClients()) {
    }

    auto ExecuteProc() -> void override {
        while (true) {
            for (auto &&item: m_GraphicClients) {
                item.second->Render();
            }

            std::this_thread::sleep_for(std::chrono::microseconds(80)); //[TODO] 改善节流控制
        }
    }
};

#endif //OSSIAN_CLIENT_GRAPHIC_WORKER
