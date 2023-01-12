/*
 * Copyright 2023 Klepsydra Technologies AG
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CACHE_LISTENER_H
#define CACHE_LISTENER_H

#include <atomic>
#include <functional>
#include <memory>
#include <mutex>
#include <thread>

#include <klepsydra/core/time_utils.h>

namespace kpsr {
namespace mem {

template<class T>
/**
 * @brief The CacheListener class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
        : cacheListenerFunction(
              std::bind(&kpsr::mem::CacheListener<T>::onEventReceived, this, std::placeholders::_1))
        , counter(0)
        , totalCopyingTime(0)
    {}

    virtual ~CacheListener() {}

    /**
     * @brief onEventReceived
     * @param event
     */
    virtual void onEventReceived(const T &event)
    {
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
    virtual void onEventReceivedRaw(std::shared_ptr<T> event)
    {
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
    virtual std::shared_ptr<T> getLastReceivedEvent() { return _lastReceivedEvent; }

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
 * @copyright 2023 Klepsydra Technologies AG
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
    void onEventReceived(const T &event) override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        CacheListener<T>::onEventReceived(event);
    }

    /**
     * @brief onEventReceived
     * @param event
     */
    void onEventReceivedRaw(std::shared_ptr<T> event) override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        CacheListener<T>::onEventReceivedRaw(event);
    }

    /**
     * @brief getLastReceivedEvent
     * @return
     */
    std::shared_ptr<T> getLastReceivedEvent() override
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return CacheListener<T>::getLastReceivedEvent();
    }

    /**
     * @brief getCounterAndEvent
     * @return
     */
    std::pair<int, std::shared_ptr<T>> getCounterAndEvent()
    {
        std::lock_guard<std::mutex> lock(_mutex);
        return std::pair<int, std::shared_ptr<T>>(this->counter,
                                                  CacheListener<T>::getLastReceivedEvent());
    }

private:
    mutable std::mutex _mutex;
};

template<class T>
/**
 * @brief The TestCacheListener class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    void onEventReceived(const T &event) override
    {
        CacheListener<T>::onEventReceived(event);
        if (_sleepTimeMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(_sleepTimeMs));
        }
    }

    /**
     * @brief onEventReceived
     * @param event
     */
    void onEventReceivedRaw(std::shared_ptr<T> event) override
    {
        CacheListener<T>::onEventReceivedRaw(event);
        if (_sleepTimeMs > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(_sleepTimeMs));
        }
    }

private:
    long _sleepTimeMs;
};

} // namespace mem
} // namespace kpsr
#endif
