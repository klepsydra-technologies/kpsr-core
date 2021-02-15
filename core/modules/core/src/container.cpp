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

#include <klepsydra/core/container.h>

kpsr::Container::Container(Environment * env,
          const std::string & applicationName)
    : _env(env)
    , _applicationName(applicationName)
    , _running(false)
{}

kpsr::Container::~Container() {
    _managedServices.clear();
    _serviceStats.clear();
    _functionStats.clear();
    _publicationStats.clear();
    _subscriptionStats.clear();
}

void kpsr::Container::start() {
    _running = true;
}

void kpsr::Container::stop() {
    _running = false;
}

void kpsr::Container::attach(Service * service) {
    if (nullptr == service) {
        return;
    }
    std::lock_guard<std::mutex> lock (_serviceMutex);
    _managedServices.push_back(service);
    _serviceStats.push_back(&service->_serviceStats);
}

void kpsr::Container::detach(Service * service) {
    if (nullptr == service) {
        return;
    }
    std::lock_guard<std::mutex> lock (_serviceMutex);
    {
        auto itr = std::find_if(_managedServices.begin(),
                                _managedServices.end(),
                                [service](Service *item) { return item->_serviceStats._name == service->_serviceStats._name;});
        if (itr != _managedServices.end()) {
            _managedServices.erase(itr);
        }
    }
    {
        auto itr = std::find_if(_serviceStats.begin(),
                                _serviceStats.end(),
                                [service](ServiceStats *item) { return item->_name == service->_serviceStats._name;});
        if (itr != _serviceStats.end()) {
            _serviceStats.erase(itr);
        }
    }
}

void kpsr::Container::attach(FunctionStats * functionStats) {
    if (nullptr == functionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock (_functionStatsMutex);
    _functionStats.push_back(functionStats);
}

void kpsr::Container::detach(FunctionStats * functionStats) {
    if (nullptr == functionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock (_functionStatsMutex);
    auto itr = std::find_if(_functionStats.begin(),
                           _functionStats.end(),
                           [functionStats](FunctionStats * item) { return item->_name == functionStats->_name; });
    if (itr != _functionStats.end()) {
        _functionStats.erase(itr);
    }
}

void kpsr::Container::attach(ServiceStats * serviceStats) {
    if (nullptr == serviceStats) {
        return;
    }
    std::lock_guard<std::mutex> lock (_serviceStatsMutex);
    _serviceStats.push_back(serviceStats);
}

void kpsr::Container::attach(PublicationStats * publicationStats) {
    if (nullptr == publicationStats) {
        return;
    }
    std::lock_guard<std::mutex> lock (_publishStatsMutex);
    _publicationStats.push_back(publicationStats);
}

void kpsr::Container::attach(SubscriptionStats * subscriptionStats) {
    if (nullptr == subscriptionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock (_subscriptionStatsMutex);
    _subscriptionStats.push_back(subscriptionStats);
}

void kpsr::Container::detach(SubscriptionStats * subscriptionStats) {
    if (nullptr == subscriptionStats) {
        return;
    }
    std::lock_guard<std::mutex> lock (_subscriptionStatsMutex);
    auto itr = std::find_if(_subscriptionStats.begin(),
                           _subscriptionStats.end(),
                           [subscriptionStats](SubscriptionStats * item) { return item->_name == subscriptionStats->_name; });
    if (itr != _subscriptionStats.end()) {
        _subscriptionStats.erase(itr);
    }
}
