/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef MANAGE_SERVICE_H
#define MANAGE_SERVICE_H

#include <atomic>
#include <map>

#include <spdlog/spdlog.h>

#include <klepsydra/core/system_event.h>
#include <klepsydra/sdk/environment.h>
#include <klepsydra/sdk/service.h>
#include <klepsydra/sdk/service_stats.h>
#include <klepsydra/sdk/subscriber.h>

namespace kpsr {
/*!
 * @brief The ManagedService class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details There are two types of services: master and slave. They are distinguised in construction
 * time by passing the parameter ''systemStatusEventSubscriber'' as null or not null respectively.
 *
 * Slave services in the other hand only start running when commanded by a system service. A system service is nothing
 * but a master service that can publish events of the System Event Data type.
 */
class ManagedService : public Service
{
public:
    /*!
     * @brief ManagedService
     * @param environment
     * @param systemStatusEventSubscriber start and stop will be handled by the master service.
     * @param serviceName
     */
    ManagedService(Environment *environment,
                   Subscriber<SystemEventData> *systemStatusEventSubscriber,
                   std::string serviceName)
        : Service(nullptr, environment, serviceName, false)
    {
        if (systemStatusEventSubscriber != nullptr) {
            std::function<void(SystemEventData)> systemListener =
                std::bind(&ManagedService::onSystemMessageReceived, this, std::placeholders::_1);
            systemStatusEventSubscriber->registerListener(serviceName, systemListener);
        }
    }

    virtual ~ManagedService() {}

private:
    void onSystemMessageReceived(const SystemEventData &event)
    {
        spdlog::info(
            "Service::SystemEventDataListener::onMessageReceived. Service name: {}, event: {}",
            this->serviceStats.name,
            event);
        if (event == SystemEventData::Start) {
            startup();
        } else if (event == SystemEventData::Stop) {
            shutdown();
        }
    }
};
} // namespace kpsr

#endif // MANAGE_SERVICE_H
