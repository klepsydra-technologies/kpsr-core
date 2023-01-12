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

#include <klepsydra/mem_core/basic_scheduler.h>

kpsr::mem::BasicScheduler::ScheduledThread::ScheduledThread(int after,
                                                            bool repeat,
                                                            std::function<void()> task)
    : _after(after)
    , _repeat(repeat)
    , _isRunning(false)
    , _task(task)
{}

void kpsr::mem::BasicScheduler::ScheduledThread::start()
{
    _isRunning = true;
    std::function<void()> scheduledTask = [this]() {
        if (_repeat) {
            while (_isRunning) {
                std::this_thread::sleep_for(std::chrono::microseconds(_after));
                _task();
            }
        } else {
            std::this_thread::sleep_for(std::chrono::microseconds(_after));
            _task();
        }
    };

    _thread = std::thread(scheduledTask);
}

void kpsr::mem::BasicScheduler::ScheduledThread::stop()
{
    _isRunning = false;
    if (_thread.joinable()) {
        _thread.join();
    }
}

void kpsr::mem::BasicScheduler::startScheduledTask(const std::string &name,
                                                   int after,
                                                   bool repeat,
                                                   std::function<void()> task)
{
    _threadMap[name] = std::shared_ptr<ScheduledThread>(new ScheduledThread(after, repeat, task));
    _threadMap[name]->start();
}

void kpsr::mem::BasicScheduler::stopScheduledTask(const std::string &name)
{
    if (_threadMap.find(name) != _threadMap.end()) {
        _threadMap[name]->stop();
        _threadMap.erase(name);
    }
}

void kpsr::mem::BasicScheduler::startScheduledService(int after, bool repeat, Service *service)
{
    std::string name = service->_serviceStats.name;
    std::function<void()> task = std::function<void()>(std::bind(&Service::runOnce, service));
    startScheduledTask(name, after, repeat, task);
}

void kpsr::mem::BasicScheduler::stopScheduledService(Service *service)
{
    std::string name = service->_serviceStats.name;
    stopScheduledTask(name);
}
