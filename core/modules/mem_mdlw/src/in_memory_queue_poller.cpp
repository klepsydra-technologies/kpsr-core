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
    _running = true;
    _threadNotifier = std::thread(&InMemoryQueuePoller::pollingLoop, this);
}

void InMemoryQueuePoller::stop() {
    _running = false;
    if(_threadNotifier.joinable()) {
        _threadNotifier.join();
    }
}

InMemoryQueuePoller::~InMemoryQueuePoller() {
    _running = false;
    if(_threadNotifier.joinable()) {
        _threadNotifier.join();
    }
}

void InMemoryQueuePoller::pollingLoop() {
    while (_running) {
        takeEventFromQueue();
    }
}

}
}
