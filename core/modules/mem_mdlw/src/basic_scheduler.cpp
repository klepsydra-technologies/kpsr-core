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

#include <klepsydra/mem_core/basic_scheduler.h>

kpsr::mem::BasicScheduler::ScheduledThread::ScheduledThread(int after, bool repeat, std::shared_ptr<std::function<void ()>> task)
    : _after(after)
    , _repeat(repeat)
    , _isRunning(false)
    , _task(task)
{}

void kpsr::mem::BasicScheduler::ScheduledThread::start() {

    _isRunning = true;
    std::function<void ()> scheduledTask = [this]() {
        if (_repeat) {
            while (_isRunning) {
                std::this_thread::sleep_for(std::chrono::microseconds(_after));
                (*_task.get())();
            }
        }
        else {
            std::this_thread::sleep_for(std::chrono::microseconds(_after));
            (*_task.get())();
        }
    };

    _thread = std::thread(scheduledTask);
}

void kpsr::mem::BasicScheduler::ScheduledThread::stop() {
    _isRunning = false;
    if(_thread.joinable()) {
        _thread.join();
    }
}

void kpsr::mem::BasicScheduler::startScheduledTask(const std::string & name, int after, bool repeat, std::shared_ptr<std::function<void ()>> task) {
    _threadMap[name] = std::shared_ptr<ScheduledThread>(new ScheduledThread(after, repeat, task));
    _threadMap[name]->start();
}

void kpsr::mem::BasicScheduler::stopScheduledTask(const std::string & name) {
    if (_threadMap.find(name) != _threadMap.end()) {
        _threadMap[name]->stop();
        _threadMap.erase(name);
    }
}

void kpsr::mem::BasicScheduler::startScheduledService(int after, bool repeat, Service * service) {
    std::string name = service->_serviceStats._name;
    std::shared_ptr<std::function<void ()>> task = std::make_shared<std::function<void ()>>(std::bind(&Service::runOnce, service));
    startScheduledTask(name, after, repeat, task);
}

void kpsr::mem::BasicScheduler::stopScheduledService(Service *service) {
    std::string name = service->_serviceStats._name;
    stopScheduledTask(name);
}
