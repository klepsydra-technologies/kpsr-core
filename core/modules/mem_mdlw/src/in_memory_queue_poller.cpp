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

#include <klepsydra/mem_core/in_memory_queue_poller.h>

namespace kpsr {
namespace mem {
void InMemoryQueuePoller::start()
{
    if (isStarted()) {
        return;
    }
    this->_started.store(true, std::memory_order_release);
    _threadNotifier = std::thread(std::move(_loopFunction));
    int counterUs = 0;
    while (!isRunning()) {
        if (counterUs > _timeoutUs) {
            throw std::runtime_error("Could not start the poller");
        }
        std::this_thread::sleep_for(std::chrono::microseconds(100));
        counterUs += 100;
    }
}

void InMemoryQueuePoller::stop()
{
    if (!isStarted()) {
        return;
    }
    _running.store(false, std::memory_order_release);
    if (_threadNotifier.joinable()) {
        _threadNotifier.join();
    }
    _loopFunction = (std::packaged_task<void()>(
        std::bind(&InMemoryQueuePoller::pollingLoop, this))); // to allow restarting
    _started.store(false, std::memory_order_release);
}

InMemoryQueuePoller::~InMemoryQueuePoller()
{
    _running = false;
    if (_threadNotifier.joinable()) {
        _threadNotifier.join();
    }
}

void InMemoryQueuePoller::pollingLoop()
{
    _running.store(true, std::memory_order_release);
    while (isRunning()) {
        takeEventFromQueue();
    }
}

bool InMemoryQueuePoller::isStarted()
{
    return _started.load(std::memory_order_acquire);
}

bool InMemoryQueuePoller::isRunning()
{
    return _running.load(std::memory_order_acquire);
}
} // namespace mem

} // namespace kpsr
