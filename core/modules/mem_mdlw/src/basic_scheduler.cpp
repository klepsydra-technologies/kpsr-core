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

void kpsr::mem::BasicScheduler::startScheduledTask(std::string name, int after, bool repeat, std::shared_ptr<std::function<void ()>> task) {
    _threadMap[name] = std::shared_ptr<ScheduledThread>(new ScheduledThread(after, repeat, task));
    _threadMap[name]->start();
}

void kpsr::mem::BasicScheduler::stopScheduledTask(std::string name) {
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
