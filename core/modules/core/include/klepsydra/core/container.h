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
    Container(Environment * env, const std::string & applicationName);

    virtual ~Container();

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
     * @brief detach
     * @param service Pointer to a service
     */
    void detach(Service * service);

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
