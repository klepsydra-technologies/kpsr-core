/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#ifndef MANAGE_SERVICE_H
#define MANAGE_SERVICE_H

#include <atomic>
#include <map>

#include "spdlog/spdlog.h"
#include "spdlog/sinks/basic_file_sink.h"

#include <klepsydra/core/environment.h>
#include <klepsydra/core/service.h>
#include <klepsydra/core/subscriber.h>
#include <klepsydra/core/system_event.h>

#include <klepsydra/core/service_stats.h>

namespace kpsr
{
/*!
 * @brief The ManagedService class
 *
 * @copyright Klepsydra Technologies 2019-2020.
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
    ManagedService(Environment * environment,
                   Subscriber<SystemEventData> * systemStatusEventSubscriber,
                   std::string serviceName)
        : Service(environment, serviceName, false) {
        if (systemStatusEventSubscriber != nullptr) {
            std::function<void(SystemEventData)> systemListener = std::bind(&ManagedService::onSystemMessageReceived, this, std::placeholders::_1);
            systemStatusEventSubscriber->registerListener(serviceName, systemListener);
        }
    }

private:

    void onSystemMessageReceived(const SystemEventData & event) {
        spdlog::info("Service::SystemEventDataListener::onMessageReceived.");
        if (event == SystemEventData::Start) {
            startup();
        }
        else if (event == SystemEventData::Stop) {
            shutdown();
        }
    }
};
}

#endif // MANAGE_SERVICE_H
