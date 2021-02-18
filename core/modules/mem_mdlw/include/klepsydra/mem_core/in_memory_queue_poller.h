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

#ifndef IN_MEMORY_QUEUE_POLLER_H
#define IN_MEMORY_QUEUE_POLLER_H

#include <map>
#include <thread>
#include <atomic>
#include <string>
#include <future>

#include <klepsydra/core/event_emitter.h>

#include <klepsydra/mem_core/basic_event_data.h>

namespace kpsr
{
namespace mem
{

static const long MEM_START_TIMEOUT_MILLISEC = 100;
    
/**
 * @brief The InMemoryQueuePoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details This class, provides the API to a class which extends from
 * the event_emitter_subscriber.h is an asynchronous in-memory
 * middleware.  It block the subcriber thread when no events are
 * availble. The polling loop depends on the type of queue being used
 * (locking or non-locking).
 *
*/
class InMemoryQueuePoller
{
public:
    /**
     * @brief InMemoryQueuePoller
     * @param eventEmitter
     * @param eventName
     * @param sleepPeriodUS The time in microseconds to sleep/wait
     */
    InMemoryQueuePoller(EventEmitter & eventEmitter,
                        std::string eventName,
                        unsigned int sleepPeriodUS,
                        long timeoutMS = MEM_START_TIMEOUT_MILLISEC)
        : _running(false)
        , _started(false)
        , _eventEmitter(eventEmitter)
        , _eventName(eventName)
        , _threadNotifier()
        , _sleepPeriodUS(sleepPeriodUS)
        , _loopFunction(std::bind(&InMemoryQueuePoller::pollingLoop, this))
        , _threadNotifierFuture(_loopFunction.get_future())
        , _timeoutUs(timeoutMS*1000)
    {}

    /**
     * @brief start
     */
    void start();

    /**
     * @brief stop
     */
    void stop();

    virtual ~InMemoryQueuePoller();

    /**
     * @brief _running
     */
    std::atomic<bool> _running;

    /**
     * @brief isRunning
     */
    bool isRunning();
private:
    std::atomic<bool> _started;
    bool isStarted();
    void pollingLoop();

    virtual void takeEventFromQueue() = 0;

protected:
    EventEmitter & _eventEmitter;
    std::string _eventName;
    std::thread _threadNotifier;
    unsigned int _sleepPeriodUS;
private:
    std::packaged_task<void()> _loopFunction;
    std::future<void> _threadNotifierFuture;
    long _timeoutUs;
};
}
}
#endif
