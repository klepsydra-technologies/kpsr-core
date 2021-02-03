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

#ifndef _EVENT_EMITTER_H_
#define _EVENT_EMITTER_H_

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <list>
#include <algorithm>

#include <klepsydra/core/subscription_stats.h>
#include <spdlog/spdlog.h>

namespace kpsr {
/**
 * @brief The EventEmitter class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details An event emitter pattern implementation oriented to Klepsydra API.
 */
class EventEmitter
{
public:
    
    /**
     * @brief EventEmitter
     */
    EventEmitter();

    ~EventEmitter();

    template <typename... Args>
    /**
     * @brief add_listener
     * @param event_id
     * @param listener_name
     * @param isOnce
     * @param cb
     * @return
     */
    unsigned int add_listener(const std::string & event_id, const std::string & listener_name, bool isOnce, std::function<void (const Args & ...)> cb);

    /**
     * @brief add_listener
     * @param event_id
     * @param listener_name
     * @param isOnce
     * @param cb
     * @return
     */
    unsigned int add_listener(const std::string & event_id, const std::string & listener_name, bool isOnce, std::function<void ()> cb);

    template <typename... Args>
    /**
     * @brief on
     * @param event_id
     * @param listener_name
     * @param cb
     * @return
     */
    unsigned int on(const std::string & event_id, const std::string & listener_name, std::function<void (const Args & ...)> cb);
    
    /**
     * @brief on
     * @param event_id
     * @param listener_name
     * @param cb
     * @return
     */
    unsigned int on(const std::string & event_id, const std::string & listener_name, std::function<void ()> cb);

    template <typename... Args>
    /**
     * @brief once
     * @param event_id
     * @param cb
     * @return
     */
    unsigned int once(const std::string & event_id, std::function<void (const Args & ...)> cb);

    /**
     * @brief once
     * @param event_id
     * @param cb
     * @return
     */
    unsigned int once(const std::string & event_id, std::function<void ()> cb);

    /**
     * @brief remove_listener
     * @param listener_id
     */
    void remove_listener(unsigned int listener_id);

    template <typename... Args>
    /**
     * @brief emitEvent
     * @param event_id
     * @param enqueuedTimeNs
     * @param args
     */
    void emitEvent(const std::string & event_id, long long unsigned int enqueuedTimeNs, const Args & ... args);

    /**
     * @brief _listenerStats
     *
     * A map that contains all the subscription stats associated to each listener in the event emitter.
     *
     */
    std::map<unsigned int, std::shared_ptr<SubscriptionStats>> _listenerStats;

private:
    struct ListenerBase
    {
        ListenerBase() {}

        ListenerBase(unsigned int i, bool isOnce)
            : id(i)
            , isOnce(isOnce)
        {}

        virtual ~ListenerBase() {}

        unsigned int id;
        bool isOnce;
    };

    template <typename... Args>
    struct Listener : public ListenerBase
    {
        Listener() {}

        Listener(unsigned int i, bool isOnce, const std::function<void (const Args & ...)> & c)
            : ListenerBase(i, isOnce), cb(c) {}

        std::function<void (const Args &...)> cb;
    };

    std::mutex mutex;
    unsigned int last_listener = 0;
    std::multimap<std::string, std::shared_ptr<ListenerBase>> listeners;

    EventEmitter(const EventEmitter&) = delete;
    const EventEmitter& operator = (const EventEmitter&) = delete;
};
}


template <typename... Args>
unsigned int kpsr::EventEmitter::add_listener(const std::string & event_id, const std::string & listener_name, bool isOnce, std::function<void (const Args & ...)> cb)
{
    if (!cb)
    {
        throw std::invalid_argument("kpsr::EventEmitter::add_listener: No callbak provided.");
    }

    std::lock_guard<std::mutex> lock(mutex);

    unsigned int listener_id = ++last_listener;
    listeners.insert(std::make_pair(event_id, std::make_shared<Listener<Args...>>(listener_id, isOnce, cb)));
    if (!isOnce) {
        _listenerStats.insert(std::make_pair(listener_id, std::make_shared<kpsr::SubscriptionStats>(listener_name, event_id, "EVENT_EMITTER")));
    }

    return listener_id;
}

template <typename... Args>
unsigned int kpsr::EventEmitter::on(const std::string & event_id, const std::string & listener_name, std::function<void (const Args & ...)> cb)
{
    return add_listener(event_id, listener_name, false, cb);
}

template <typename... Args>
unsigned int kpsr::EventEmitter::once(const std::string & event_id, std::function<void (const Args & ...)> cb)
{
    return add_listener(event_id, "once", true, cb);
}

template <typename... Args>
void kpsr::EventEmitter::emitEvent(const std::string & event_id, long long unsigned int enqueuedTimeNs, const Args & ... args)
{
    std::vector<std::shared_ptr<Listener<Args...>>> handlers(listeners.size());
    {
        std::lock_guard<std::mutex> lock(mutex);

        auto range = listeners.equal_range(event_id);
        handlers.resize(std::distance(range.first, range.second));
        std::transform(range.first, range.second, handlers.begin(), [] (const std::pair<const std::string, std::shared_ptr<ListenerBase>> &p) {
            auto l = std::dynamic_pointer_cast<Listener<Args...>>(p.second);
            if (l)
            {
                return l;
            }
            else
            {
                throw std::logic_error("kpsr::EventEmitter::emitEvent: Invalid event signature.");
            }
        });
    }

    for (auto& h : handlers)
    {
        if (h->isOnce) {
            h->cb(args...);
            remove_listener(h->id);
            continue;
        }

        try {
            auto listenerStatistic = _listenerStats.at(h->id);
            if (!listenerStatistic) {
                continue;
            }
            listenerStatistic->startProcessMeassure();
            if (enqueuedTimeNs > 0) {
                listenerStatistic->_totalEnqueuedTimeInNs += (TimeUtils::getCurrentNanosecondsAsLlu() - enqueuedTimeNs);
            }
            h->cb(args...);
            listenerStatistic->stopProcessMeassure();
        } catch (std::out_of_range &ex) {
            // Nothing to do. If statics are removed then the handler & callback function have also been removed from map.
            spdlog::info("Listener already removed from list");
        }
    }
}

#endif

