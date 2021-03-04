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

#ifndef ZMQ_POLLER_H
#define ZMQ_POLLER_H

#include <string>
#include <functional>
#include <atomic>
#include <mutex>
#include <future>
#include <map>

namespace kpsr {
namespace zmq_mdlw {

static const long ZMQ_START_TIMEOUT_MILLISEC = 100;
    
template<class T>
/**
 * @brief The ZMQPoller class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
class ZMQPoller {
public:
    /**
     * @brief ZMQPoller
     * @param subscriber
     * @param pollPeriod
     */
    ZMQPoller(zmq::socket_t & subscriber, long pollPeriod,
              long timeoutMS = ZMQ_START_TIMEOUT_MILLISEC)
        : _subscriber(subscriber)
        , _pollPeriod(pollPeriod)
        , _threadNotifier()
        , _running(false)
        , _started(false)
        , _poller(std::bind(&ZMQPoller::pollingLoop, this))
        , _threadNotifierFuture(_poller.get_future())
        , _timeoutUs(timeoutMS*1000)
    {}

    /**
     * @brief start
     */
    virtual void start() {
        if (isStarted()) {
            return;
        }
        this->_started.store(true, std::memory_order_release);
        _threadNotifier = std::thread(std::move(_poller));
        long counterUs = 0;
        while (!isRunning()) {
            if (counterUs > _timeoutUs) {
                throw std::runtime_error("Could not start the ZMQ poller");
            }
            std::this_thread::sleep_for(std::chrono::microseconds(100));
            counterUs += 100;
        }
    }

    /**
     * @brief stop
     */
    virtual void stop() {
        if (!isStarted()) {
            return;
        }
        _running.store(false, std::memory_order_release);
        if(_threadNotifier.joinable()) {
		    _threadNotifier.join();
        }
        _poller = std::packaged_task<void()>(std::bind(&ZMQPoller::pollingLoop, this));
        _started.store(false, std::memory_order_release);
    }

    ~ZMQPoller() {
        _running = false;
        if(_threadNotifier.joinable()) {
		    _threadNotifier.join();
        }
    }

    /**
     * @brief registerToTopic
     * @param topic
     * @param function
     */
    void registerToTopic(const std::string & topic, std::function<void(const T &)> function) {
        std::lock_guard<std::mutex> lock(mutex);
        _functionTopicMap[topic] = function;
    }

    /**
     * @brief unregisterFromTopic
     * @param topic
     */
    void unregisterFromTopic(const std::string & topic) {
        std::lock_guard<std::mutex> lock(mutex);
        _functionTopicMap.erase(topic);
    }

    /**
     * @brief executeFunction
     * @param topic
     * @param event
     */
    void executeFunction(const std::string & topic, const T & event) {
        std::lock_guard<std::mutex> lock(mutex);
        auto search = _functionTopicMap.find(topic);
        if (search != _functionTopicMap.end()) {
            _functionTopicMap[topic](event);
        }
    }

    /**
     * @brief poll
     */
    virtual void poll() = 0;

protected:
    void pollingLoop() {
        _running.store(true, std::memory_order_relaxed);
        this->poll();
    }

    /**
     * @brief isRunning
     */
    bool isRunning() {
        return _running.load(std::memory_order_acquire);
    }

    /**
     * @brief isStarted
     */
    bool isStarted() {
        return _started.load(std::memory_order_acquire);
    }

    zmq::socket_t & _subscriber;
    long _pollPeriod;
    std::thread _threadNotifier;
    std::atomic<bool> _running;

    std::atomic<bool> _started;
    std::packaged_task<void()> _poller;
    std::future<void> _threadNotifierFuture;
    std::mutex mutex;

    std::map<std::string, std::function<void(const T &)>> _functionTopicMap;
    typedef typename std::map<std::string, std::function<void(const T &)>>::iterator it_type;
    long _timeoutUs;
};
}
}

#endif // ZMQ_POLLER_H
