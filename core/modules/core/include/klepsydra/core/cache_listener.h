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

#ifndef CACHE_LISTENER_H
#define CACHE_LISTENER_H

#include <functional>
#include <thread>
#include <mutex>
#include <atomic>
#include <memory>

#include <klepsydra/core/time_utils.h>

namespace kpsr
{
namespace mem
{

template<class T>
/**
 * @brief The CacheListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-test
 *
 * @details This class is part of the API facility framework build in Klepsydra.
 * The CacheListener stores a copy of the last received event. It offers also an std::function that
 * can be plugged directly into the subscriber. For example:
@code
Subscriber<std::string> subscriber;
CacheListener<std::string> cacheListener;
subscriber.registerListener("test-listener", cacheListener.cacheListenerFunction);
@endcode
 * Finally, it also keeps count of the number of event received and total time spent in copying.
 *
 */
class CacheListener
{
public:

    /**
     * @brief CacheListener
     * @param sleepTimeMs When a value larger than 0 is passed, it will sleep after copying the event.
     */
    CacheListener()
        : cacheListenerFunction(std::bind(
                                    &kpsr::mem::CacheListener<T>::onEventReceived,
                                    this,
                                    std::placeholders::_1))
        , counter(0)
        , totalCopyingTime(0)
    {}

    virtual ~CacheListener() {}

    /**
     * @brief onEventReceived
     * @param event
     */
    virtual void onEventReceived(const T & event) {
        long before = kpsr::TimeUtils::getCurrentNanoseconds();

        this->_lastReceivedEvent = std::make_shared<T>(event);

        long after = kpsr::TimeUtils::getCurrentNanoseconds();

        totalCopyingTime += after - before;
        counter++;
    }

    /**
     * @brief onEventReceived
     * @param event
     */
    virtual void onEventReceivedRaw(std::shared_ptr<T> event) {
        this->_lastReceivedEvent = event;
        counter++;
    }

    /**
     * @brief cacheListenerFunction
     */
    std::function<void(const T &)> cacheListenerFunction;

    /**
     * @brief getLastReceivedEvent
     * @return
     */
    virtual std::shared_ptr<T> getLastReceivedEvent() {
        return _lastReceivedEvent;
    }

    /**
     * @brief counter
     */
    std::atomic_int counter;

    /**
     * @brief totalCopyingTime
     */
    long totalCopyingTime;

private:
    std::shared_ptr<T> _lastReceivedEvent;

    bool _threadSafe;
    mutable std::mutex _mutex;

};


template<class T>
/**
 * @brief The MultiThreadCacheListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-test
 *
 * @details This class is part of the API facility framework build in Klepsydra.
 * The MultiThreadCacheListener stores a copy of the last received event in a thread-safe manner. It offers
 * also an std::function that can be plugged directly into the subscriber. For example:
@code
Subscriber<std::string> subscriber;
MultiThreadCacheListener<std::string> cacheListener;
subscriber.registerListener("test-listener", cacheListener.cacheListenerFunction);
@endcode
 *
 */
class MultiThreadCacheListener : public CacheListener<T>
{
public:

    /**
     * @brief CacheListener
     * @param sleepTimeMs When a value larger than 0 is passed, it will sleep after copying the event.
     */
    MultiThreadCacheListener()
        : CacheListener<T>()
    {}

    virtual ~MultiThreadCacheListener() {}

    /**
     * @brief onEventReceived
     * @param event
     */
    void onEventReceived(const T & event) override {
        std::lock_guard<std::mutex> lock (_mutex);
        CacheListener<T>::onEventReceived(event);
    }

    /**
     * @brief onEventReceived
     * @param event
     */
    void onEventReceivedRaw(std::shared_ptr<T> event) override {
        std::lock_guard<std::mutex> lock (_mutex);
        CacheListener<T>::onEventReceivedRaw(event);
    }

    /**
     * @brief getLastReceivedEvent
     * @return
     */
    std::shared_ptr<T> getLastReceivedEvent() override {
        std::lock_guard<std::mutex> lock (_mutex);
        return CacheListener<T>::getLastReceivedEvent();
    }

    /**
     * @brief getCounterAndEvent
     * @return
     */
    std::pair<int, std::shared_ptr<T>> getCounterAndEvent() {
        std::lock_guard<std::mutex> lock (_mutex);
        return std::pair<int, std::shared_ptr<T>>(this->counter, CacheListener<T>::getLastReceivedEvent());
    }

private:
    mutable std::mutex _mutex;

};


template<class T>
/**
 * @brief The TestCacheListener class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-test
 *
 * @details This class is part of the test facility framework build in Klepsydra.
 * The CacheListener stores a copy of the last received event. It offers also an std::function that can
 * be plugged directly into the subscriber. For example:
@code
Subscriber<std::string> subscriber;
TestCacheListener<std::string> cacheListener;
subscriber.registerListener("test-listener", cacheListener.cacheListenerFunction);
@endcode
 * It can also sleep for a configured number of milliseconds as a way to mock processing time.
 * Finally, it also keeps count of the number of event received and total time spent in copying.
 *
 */
class TestCacheListener : public CacheListener<T>
{
public:

    /**
     * @brief CacheListener
     * @param sleepTimeMs When a value larger than 0 is passed, it will sleep after copying the event.
     */
    TestCacheListener(int sleepTimeMs)
        : CacheListener<T>()
        , _sleepTimeMs(sleepTimeMs)
    {}

    /**
     * @brief onEventReceived
     * @param event
     */
    void onEventReceived(const T & event) override {
        CacheListener<T>::onEventReceived(event);
        if (_sleepTimeMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(_sleepTimeMs));
        }
    }

    /**
     * @brief onEventReceived
     * @param event
     */
    void onEventReceivedRaw(std::shared_ptr<T> event) override {
        CacheListener<T>::onEventReceivedRaw(event);
        if (_sleepTimeMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(_sleepTimeMs));
        }
    }

private:
    long _sleepTimeMs;
};

}
}
#endif

