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

#ifndef CONTAINER_H
#define CONTAINER_H

#include <string>
#include <vector>
#include <atomic>
#include <algorithm>
#include <mutex>

#include <klepsydra/core/publication_stats.h>
#include <klepsydra/core/subscription_stats.h>
#include <klepsydra/core/service_stats.h>
#include <klepsydra/core/service.h>

namespace kpsr
{
/**
 * @brief The Container class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-monitoring
 *
 * @details This class keeps a reference to the statistics object in each service. subscriber and publisher in the system. Custom methods statistics can
 * also be registered here.
 *
 * Services, publishers and subscriber will register themselves when they receive a non-null pointer to a container in their constructor.
 *
 */
class Container
{
public:
    /**
     * @brief Container constructor
     * @param env to administer
     * @param applicationName to use as id of the process running these services.
     */
    Container(Environment * env, std::string applicationName);

    /**
     * @brief start all master services.
     */
    virtual void start();

    /**
     * @brief stop all services.
     */
    virtual void stop();

    /**
     * @brief attach a service and its statistics variable to the container
     * @param service Pointer to a service
     */
    void attach(Service * service);

    /**
     * @brief attach a custom method statistics to the container
     * @param functionStats Pointer to a basic stat object.
     */
    void attach(FunctionStats * functionStats);

    /**
     * @brief detach
     * @param functionStats
     */
    void detach(FunctionStats * functionStats);

    /**
     * @brief attach a service statistics to the container
     * @param serviceStats Pointer to a service stats.
     */
    void attach(ServiceStats * serviceStats);

    /**
     * @brief attach a publication statistics to the container
     * @param publicationStats Pointer to a publication stats.
     */
    void attach(PublicationStats * publicationStats);

    /**
     * @brief attach a subscription statistics to the container
     * @param subscriptionStats Pointer to a subscription stats.
     */
    void attach(SubscriptionStats * subscriptionStats);

    /**
     * @brief detach
     * @param subscriptionStats
     */
    void detach(SubscriptionStats * subscriptionStats);


protected:
    Environment * _env;
    std::string _applicationName;
    std::atomic_bool _running;

    std::vector<Service *> _managedServices;
    std::vector<FunctionStats *> _functionStats;
    std::vector<ServiceStats *> _serviceStats;
    std::vector<PublicationStats *> _publicationStats;
    std::vector<SubscriptionStats *> _subscriptionStats;
    mutable std::mutex _serviceMutex;
    mutable std::mutex _serviceStatsMutex;
    mutable std::mutex _publishStatsMutex;
    mutable std::mutex _subscriptionStatsMutex;
    mutable std::mutex _functionStatsMutex;
};
}

#endif // CONTAINER_H
