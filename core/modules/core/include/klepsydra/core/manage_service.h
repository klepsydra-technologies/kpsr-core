/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*                            All Rights Reserved.
*
*  This file is subject to the terms and conditions defined in
*  file 'LICENSE.md', which is part of this source code package.
*
*  NOTICE:  All information contained herein is, and remains the property of Klepsydra
*  Technologies GmbH and its suppliers, if any. The intellectual and technical concepts
*  contained herein are proprietary to Klepsydra Technologies GmbH and its suppliers and
*  may be covered by Swiss and Foreign Patents, patents in process, and are protected by
*  trade secret or copyright law. Dissemination of this information or reproduction of
*  this material is strictly forbidden unless prior written permission is obtained from
*  Klepsydra Technologies GmbH.
*
****************************************************************************/

#ifndef MANAGE_SERVICE_H
#define MANAGE_SERVICE_H

#include <atomic>
#include <map>

#include <spdlog/spdlog.h>


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

    virtual ~ManagedService() {}
private:

    void onSystemMessageReceived(const SystemEventData & event) {
        spdlog::info("Service::SystemEventDataListener::onMessageReceived. Service name: {}, event: {}", this->_serviceStats._name, event);
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
