/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#ifndef ZMQ_POLLER_H
#define ZMQ_POLLER_H

#include <string>
#include <functional>
#include <atomic>
#include <mutex>

namespace kpsr {
namespace zmq_mdlw {
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
    ZMQPoller(zmq::socket_t & subscriber, long pollPeriod)
        : _subscriber(subscriber)
        , _pollPeriod(pollPeriod)
        , _running(false)
        , _threadNotifier()
    {}

    /**
     * @brief start
     */
    virtual void start() {
        _running = true;
        _threadNotifier = std::thread(&ZMQPoller::poll, this);
    }

    /**
     * @brief stop
     */
    virtual void stop() {
        _running = false;
        if(_threadNotifier.joinable()) _threadNotifier.join();
    }

    ~ZMQPoller() {
        _running = false;
        if(_threadNotifier.joinable()) _threadNotifier.join();
    }

    /**
     * @brief registerToTopic
     * @param topic
     * @param function
     */
    void registerToTopic(std::string topic, std::function<void(const T &)> function) {
        std::lock_guard<std::mutex> lock(mutex);
        _functionTopicMap[topic] = function;
    }

    /**
     * @brief unregisterFromTopic
     * @param topic
     */
    void unregisterFromTopic(std::string topic) {
        std::lock_guard<std::mutex> lock(mutex);
        _functionTopicMap.erase(topic);
    }

    /**
     * @brief executeFunction
     * @param topic
     * @param event
     */
    void executeFunction(std::string topic, const T & event) {
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
    zmq::socket_t & _subscriber;
    long _pollPeriod;
    std::atomic<bool> _running;

    std::thread _threadNotifier;
    std::mutex mutex;

    std::map<std::string, std::function<void(const T &)>> _functionTopicMap;
    typedef typename std::map<std::string, std::function<void(const T &)>>::iterator it_type;

};
}
}

#endif // ZMQ_POLLER_H
