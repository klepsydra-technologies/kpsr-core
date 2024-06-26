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
#include <spdlog/spdlog.h>

kpsr::mem::BasicScheduler::ScheduledThread::ScheduledThread(const std::string &name,
                                                            int after,
                                                            bool repeat,
                                                            std::function<void()> task)
    : _name(name)
    , _after(after)
    , _repeat(repeat)
    , _isRunning(false)
    , _task(task)
{}

void kpsr::mem::BasicScheduler::ScheduledThread::start()
{
    _isRunning = true;
    std::function<void()> scheduledTask = [this]() {
        if (_repeat) {
            long long before = 0;
            long long after = 0;
            while (_isRunning) {
                int effectiveSleepTime = _after - (after - before);
                if (effectiveSleepTime > 0) {
                    std::this_thread::sleep_for(std::chrono::microseconds(effectiveSleepTime));
                } else {
                    spdlog::debug(
                        "No sleep due to long execution time. Effective sleep time is: {}",
                        effectiveSleepTime);
                }
                before = std::chrono::duration_cast<std::chrono::microseconds>(
                             std::chrono::system_clock::now().time_since_epoch())
                             .count();
                _task();
                after = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::system_clock::now().time_since_epoch())
                            .count();
            }
        } else {
            if (_isRunning) {
                std::this_thread::sleep_for(std::chrono::microseconds(_after));
                _task();
            }
        }
    };

    _thread = std::thread(std::move(scheduledTask));
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
    _threadMap[name] = std::shared_ptr<ScheduledThread>(
        new ScheduledThread(name, after, repeat, task));
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
    std::string name = service->serviceStats.name;
    std::function<void()> task = std::function<void()>(std::bind(&Service::runOnce, service));
    startScheduledTask(name, after, repeat, task);
}

void kpsr::mem::BasicScheduler::stopScheduledService(Service *service)
{
    std::string name = service->serviceStats.name;
    stopScheduledTask(name);
}
