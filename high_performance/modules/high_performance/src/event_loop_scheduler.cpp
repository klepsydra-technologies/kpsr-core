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

#include <klepsydra/high_performance/event_loop_scheduler.h>

kpsr::high_performance::EventLoopScheduler::EventLoopScheduler(Publisher<std::function<void()>> * publisher)
    : _publisher(publisher)
{}

void kpsr::high_performance::EventLoopScheduler::startScheduledTask(const std::string & name, int after, bool repeat, std::shared_ptr<std::function<void()>> task) {
    std::shared_ptr<std::function<void()>> eventloopTask = std::make_shared<std::function<void()>>([task, this] () { _publisher->publish(task); });
    _decorableScheduler.startScheduledTask(name, after, repeat, eventloopTask);
}

void kpsr::high_performance::EventLoopScheduler::startScheduledService(int after, bool repeat, Service * service) {
    std::string name = service->_serviceStats._name;
    std::shared_ptr<std::function<void()>> task = std::make_shared<std::function<void()>>(std::bind(&Service::runOnce, service));
    startScheduledTask(name, after, repeat, task);
}

void kpsr::high_performance::EventLoopScheduler::stopScheduledTask(const std::string & name) {
    _decorableScheduler.stopScheduledTask(name);
}

void kpsr::high_performance::EventLoopScheduler::stopScheduledService(Service *service) {
    _decorableScheduler.stopScheduledService(service);
}
