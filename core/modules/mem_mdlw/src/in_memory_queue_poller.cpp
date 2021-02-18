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

#include <klepsydra/mem_core/in_memory_queue_poller.h>

namespace kpsr
{
namespace mem
{
    void InMemoryQueuePoller::start() {
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

    void InMemoryQueuePoller::stop() {
        if (!isStarted()) {
            return;
        }
        _running.store(false, std::memory_order_release);
        if(_threadNotifier.joinable()) {
            _threadNotifier.join();
        }
        _loopFunction = (std::packaged_task<void()>(std::bind(&InMemoryQueuePoller::pollingLoop, this))); // to allow restarting
        _started.store(false, std::memory_order_release);
    }

    InMemoryQueuePoller::~InMemoryQueuePoller() {
        _running = false;
        if(_threadNotifier.joinable()) {
            _threadNotifier.join();
        }
    }

    void InMemoryQueuePoller::pollingLoop() {
        _running.store(true, std::memory_order_release);
        while (isRunning()) {
            takeEventFromQueue();
        }
    }

    bool InMemoryQueuePoller::isStarted() {
        return _started.load(std::memory_order_acquire);
    }

    bool InMemoryQueuePoller::isRunning() {
        return _running.load(std::memory_order_acquire);
    }
}

}
