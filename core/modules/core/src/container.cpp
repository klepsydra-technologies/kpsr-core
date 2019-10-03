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

#include <klepsydra/core/container.h>

kpsr::Container::Container(Environment * env,
          std::string applicationName)
    : _env(env)
    , _applicationName(applicationName)
    , _running(false)
{}

void kpsr::Container::start() {
    _running = true;
}

void kpsr::Container::stop() {
    _running = false;
}

void kpsr::Container::attach(Service * service) {
    std::lock_guard<std::mutex> lock (_serviceMutex);
    _managedServices.push_back(service);
    _serviceStats.push_back(&service->_serviceStats);
}

void kpsr::Container::attach(FunctionStats * functionStats) {
    std::lock_guard<std::mutex> lock (_functionStatsMutex);
    _functionStats.push_back(functionStats);
}

void kpsr::Container::detach(FunctionStats * functionStats) {
    std::lock_guard<std::mutex> lock (_functionStatsMutex);
    auto itr = std::find_if(_functionStats.begin(),
                           _functionStats.end(),
                           [functionStats](FunctionStats * item) { return item->_name == functionStats->_name; });
    if (itr != _functionStats.end()) {
        _functionStats.erase(itr);
    }
}

void kpsr::Container::attach(ServiceStats * serviceStats) {
    std::lock_guard<std::mutex> lock (_serviceStatsMutex);
    _serviceStats.push_back(serviceStats);
}

void kpsr::Container::attach(PublicationStats * publicationStats) {
    std::lock_guard<std::mutex> lock (_publishStatsMutex);
    _publicationStats.push_back(publicationStats);
}

void kpsr::Container::attach(SubscriptionStats * subscriptionStats) {
    std::lock_guard<std::mutex> lock (_subscriptionStatsMutex);
    _subscriptionStats.push_back(subscriptionStats);
}

void kpsr::Container::detach(SubscriptionStats * subscriptionStats) {
    std::lock_guard<std::mutex> lock (_subscriptionStatsMutex);
    auto itr = std::find_if(_subscriptionStats.begin(),
                           _subscriptionStats.end(),
                           [subscriptionStats](SubscriptionStats * item) { return item->_name == subscriptionStats->_name; });
    if (itr != _subscriptionStats.end()) {
        _subscriptionStats.erase(itr);
    }
}
