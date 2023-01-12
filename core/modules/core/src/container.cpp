// Copyright 2023 Klepsydra Technologies AG
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <klepsydra/core/container.h>

#include <spdlog/spdlog.h>

kpsr::Container::Container(Environment *env, const std::string &applicationName)
    : _env(env)
    , _applicationName(applicationName)
    , _running(false)
{}

kpsr::Container::~Container()
{
    _managedServices.clear();
    _serviceStats.clear();
    _functionStats.clear();
    _publicationStats.clear();
    _subscriptionStats.clear();
}

void kpsr::Container::start()
{
    _running = true;

    for (auto stats : _serviceStats) {
        stats->start();
    }

    for (auto stats : _functionStats) {
        stats->start();
    }

    for (auto stats : _publicationStats) {
        stats->start();
    }

    for (auto stats : _subscriptionStats) {
        stats->start();
    }
}

void kpsr::Container::stop()
{
    _running = false;

    for (auto stats : _serviceStats) {
        stats->stop();
    }

    for (auto stats : _functionStats) {
        stats->stop();
    }

    for (auto stats : _publicationStats) {
        stats->stop();
    }

    for (auto stats : _subscriptionStats) {
        stats->stop();
    }
}

void kpsr::Container::attach(Service *service)
{
    if (nullptr == service) {
        return;
    }
    std::lock_guard<std::mutex> lock(_serviceMutex);
    _managedServices.push_back(service);
    _serviceStats.push_back(&service->_serviceStats);
    if (_running) {
        spdlog::info(
            "Attaching service {} after container {} started. Statistics might not be accurate.",
            service->_serviceStats.name,
            _applicationName);
        service->_serviceStats.start();
    }
}

void kpsr::Container::detach(Service *service)
{
    if (nullptr == service) {
        return;
    }
    std::lock_guard<std::mutex> lock(_serviceMutex);
    {
        auto itr = std::find_if(_managedServices.begin(),
                                _managedServices.end(),
                                [service](Service *item) {
                                    return item->_serviceStats.name == service->_serviceStats.name;
                                });
        if (itr != _managedServices.end()) {
            _managedServices.erase(itr);
        }
    }
    {
        auto itr = std::find_if(_serviceStats.begin(),
                                _serviceStats.end(),
                                [service](ServiceStats *item) {
                                    return item->name == service->_serviceStats.name;
                                });
        if (itr != _serviceStats.end()) {
            _serviceStats.erase(itr);
        }
    }
}

void kpsr::Container::attach(FunctionStats *functionStats)
{
    if (nullptr == functionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock(_functionStatsMutex);
    _functionStats.push_back(functionStats);
    if (_running) {
        spdlog::info(
            "Attaching function {} after container {} started. Statistics might not be accurate.",
            functionStats->name,
            _applicationName);
        functionStats->start();
    }
}

void kpsr::Container::detach(FunctionStats *functionStats)
{
    if (nullptr == functionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock(_functionStatsMutex);
    auto itr = std::find_if(_functionStats.begin(),
                            _functionStats.end(),
                            [functionStats](FunctionStats *item) {
                                return item->name == functionStats->name;
                            });
    if (itr != _functionStats.end()) {
        _functionStats.erase(itr);
    }
}

void kpsr::Container::attach(ServiceStats *serviceStats)
{
    if (nullptr == serviceStats) {
        return;
    }
    std::lock_guard<std::mutex> lock(_serviceStatsMutex);
    _serviceStats.push_back(serviceStats);
    if (_running) {
        spdlog::info(
            "Attaching service {} after container {} started. Statistics might not be accurate.",
            serviceStats->name,
            _applicationName);
        serviceStats->start();
    }
}

void kpsr::Container::attach(PublicationStats *publicationStats)
{
    if (nullptr == publicationStats) {
        return;
    }
    std::lock_guard<std::mutex> lock(_publishStatsMutex);
    _publicationStats.push_back(publicationStats);
    if (_running) {
        spdlog::info(
            "Attaching publisher {} after container {} started. Statistics might not be accurate.",
            publicationStats->name,
            _applicationName);
        publicationStats->start();
    }
}

void kpsr::Container::attach(SubscriptionStats *subscriptionStats)
{
    if (nullptr == subscriptionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock(_subscriptionStatsMutex);
    _subscriptionStats.push_back(subscriptionStats);
    if (_running) {
        spdlog::info(
            "Attaching subscriber {} after container {} started. Statistics might not be accurate.",
            subscriptionStats->name,
            _applicationName);
        subscriptionStats->start();
    }
}

void kpsr::Container::detach(SubscriptionStats *subscriptionStats)
{
    if (nullptr == subscriptionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock(_subscriptionStatsMutex);
    auto itr = std::find_if(_subscriptionStats.begin(),
                            _subscriptionStats.end(),
                            [subscriptionStats](SubscriptionStats *item) {
                                return item->name == subscriptionStats->name;
                            });
    if (itr != _subscriptionStats.end()) {
        _subscriptionStats.erase(itr);
    }
}
