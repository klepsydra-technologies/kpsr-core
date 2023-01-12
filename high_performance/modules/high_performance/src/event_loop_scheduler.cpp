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

#include <klepsydra/high_performance/event_loop_scheduler.h>

kpsr::high_performance::EventLoopScheduler::EventLoopScheduler(
    Publisher<std::function<void()>> *publisher)
    : _publisher(publisher)
{}

void kpsr::high_performance::EventLoopScheduler::startScheduledTask(const std::string &name,
                                                                    int after,
                                                                    bool repeat,
                                                                    std::function<void()> task)
{
    std::function<void()> eventloopTask = std::function<void()>(
        [task, this]() { _publisher->publish(task); });
    _decorableScheduler.startScheduledTask(name, after, repeat, eventloopTask);
}

void kpsr::high_performance::EventLoopScheduler::startScheduledService(int after,
                                                                       bool repeat,
                                                                       Service *service)
{
    std::string name = service->_serviceStats.name;
    std::function<void()> task = std::function<void()>(std::bind(&Service::runOnce, service));
    startScheduledTask(name, after, repeat, task);
}

void kpsr::high_performance::EventLoopScheduler::stopScheduledTask(const std::string &name)
{
    _decorableScheduler.stopScheduledTask(name);
}

void kpsr::high_performance::EventLoopScheduler::stopScheduledService(Service *service)
{
    _decorableScheduler.stopScheduledService(service);
}
