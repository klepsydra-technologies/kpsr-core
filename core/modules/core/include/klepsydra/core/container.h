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

#ifndef CONTAINER_H
#define CONTAINER_H

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string>
#include <vector>

#include <klepsydra/core/publication_stats.h>
#include <klepsydra/core/service.h>
#include <klepsydra/core/service_stats.h>
#include <klepsydra/core/subscription_stats.h>

namespace kpsr {
/**
 * @brief The Container class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    Container(Environment *env, const std::string &applicationName);

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
    void attach(Service *service);

    /**
     * @brief detach
     * @param service Pointer to a service
     */
    void detach(Service *service);

    /**
     * @brief attach a custom method statistics to the container
     * @param functionStats Pointer to a basic stat object.
     */
    void attach(FunctionStats *functionStats);

    /**
     * @brief detach
     * @param functionStats
     */
    void detach(FunctionStats *functionStats);

    /**
     * @brief attach a service statistics to the container
     * @param serviceStats Pointer to a service stats.
     */
    void attach(ServiceStats *serviceStats);

    /**
     * @brief attach a publication statistics to the container
     * @param publicationStats Pointer to a publication stats.
     */
    void attach(PublicationStats *publicationStats);

    /**
     * @brief attach a subscription statistics to the container
     * @param subscriptionStats Pointer to a subscription stats.
     */
    void attach(SubscriptionStats *subscriptionStats);

    /**
     * @brief detach
     * @param subscriptionStats
     */
    void detach(SubscriptionStats *subscriptionStats);

protected:
    Environment *_env;
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
} // namespace kpsr

#endif // CONTAINER_H
