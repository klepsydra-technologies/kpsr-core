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

#include <klepsydra/high_performance/event_loop_scheduler.h>

kpsr::high_performance::EventLoopScheduler::EventLoopScheduler(Publisher<std::function<void()>> * publisher)
    : _publisher(publisher)
{}

void kpsr::high_performance::EventLoopScheduler::startScheduledTask(std::string name, int after, bool repeat, std::shared_ptr<std::function<void()>> task) {
    std::shared_ptr<std::function<void()>> eventloopTask = std::make_shared<std::function<void()>>([task, this] () { _publisher->publish(task); });
    _decorableScheduler.startScheduledTask(name, after, repeat, eventloopTask);
}

void kpsr::high_performance::EventLoopScheduler::startScheduledService(int after, bool repeat, Service * service) {
    std::string name = service->_serviceStats._name;
    std::shared_ptr<std::function<void()>> task = std::make_shared<std::function<void()>>(std::bind(&Service::runOnce, service));
    startScheduledTask(name, after, repeat, task);
}

void kpsr::high_performance::EventLoopScheduler::stopScheduledTask(std::string name) {
    _decorableScheduler.stopScheduledTask(name);
}

void kpsr::high_performance::EventLoopScheduler::stopScheduledService(Service *service) {
    _decorableScheduler.stopScheduledService(service);
}
