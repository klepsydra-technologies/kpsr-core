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

#ifndef CONCURRENT_QUEUE_POLLER_H
#define CONCURRENT_QUEUE_POLLER_H

#include <map>
#include <thread>
#include <atomic>
#include <string>

#include <klepsydra/core/event_emitter.h>

#include <klepsydra/mem_core/basic_event_data.h>
#include <klepsydra/mem_core/in_memory_queue_poller.h>

#include <concurrentqueue.h>

namespace kpsr
{
namespace mem
{
template <class T>
/**
 * @brief The ConcurrentQueuePoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class, which extends from the
 * event_emitter_subscriber.h is an asynchronous in-memory middleware.
 * It blocks the subcriber thread when no events are available. This
 * class uses the non-locking concurrent queue.
 *
*/
class ConcurrentQueuePoller : public InMemoryQueuePoller
{
public:
    /**
     * @brief ConcurrentQueuePoller
     * @param concurrentQueue
     * @param eventEmitter
     * @param eventName
     * @param sleepPeriodUS The time in microseconds to sleep/wait
     * @param token The producer token used by the publisher
     */
    ConcurrentQueuePoller(moodycamel::ConcurrentQueue <EventData<const T>> & concurrentQueue,
                          EventEmitter & eventEmitter,
                          std::string eventName,
                          unsigned int sleepPeriodUS,
                          moodycamel::ProducerToken & token)
        : InMemoryQueuePoller(eventEmitter, eventName, sleepPeriodUS)
        , _internalQueue(concurrentQueue)
        , _token(token)
    {}

private:

    void takeEventFromQueue() override {
        EventData<const T> event;
        bool ok = _internalQueue.try_dequeue_from_producer(_token, event);
        if (ok) {
            _eventEmitter.emitEvent(_eventName, event.enqueuedTimeInNs, * event.eventData.get());
        }
        else {
            std::this_thread::sleep_for(std::chrono::microseconds(_sleepPeriodUS));
        }
    }


    moodycamel::ConcurrentQueue <EventData<const T>> &_internalQueue;
    moodycamel::ProducerToken & _token;
};
}
}
#endif
