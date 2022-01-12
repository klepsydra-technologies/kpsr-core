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

#ifndef SAFE_EVENT_EMITTER_H_
#define SAFE_EVENT_EMITTER_H_

#include <algorithm>
#include <list>
#include <map>
#include <mutex>

#include <spdlog/spdlog.h>

#include <klepsydra/core/event_emitter_interface.h>

namespace kpsr {
/**
 * @brief The SafeEventEmitter class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-composition
 *
 * @details An event emitter pattern implementation oriented to Klepsydra API.
 */
template<typename Event>
class SafeEventEmitter : public EventEmitterInterface<Event>
{
public:
    /**
     * @brief SafeEventEmitter
     */
    SafeEventEmitter() {}

    ~SafeEventEmitter() {}

    /**
     * @brief on
     * @param listenerName
     * @param subscriberName
     * @param callback
     * @return
     */
    unsigned int on(Container *container,
                    const std::string &listenerName,
                    const std::string &subscriberName,
                    std::function<void(const Event &event)> callback) override
    {
        return addListener(container, listenerName, subscriberName, false, callback);
    }

    /**
     * @brief once
     * @param subscriberName
     * @param callback
     * @return
     */
    unsigned int once(const std::string &subscriberName,
                      std::function<void(const Event &event)> callback) override
    {
        return addListener(nullptr, SINGLE_EXECUTING_LISTENER, subscriberName, true, callback);
    }

    /**
     * @brief removeListener
     * @param listenerId
     */
    void removeListener(Container *container, unsigned int listenerId) override
    {
        std::lock_guard<std::mutex> lock(mutex);

        auto i = std::find_if(listeners.begin(),
                              listeners.end(),
                              [&](std::pair<const std::string, std::shared_ptr<ListenerBase>> p) {
                                  return p.second->id == listenerId;
                              });
        if (i != listeners.end()) {
            auto &listenerBasePtr = i->second;
            if ((container != nullptr) && (listenerBasePtr->listenerStats)) {
                container->detach(listenerBasePtr->listenerStats.get());
            }
            listeners.erase(i);
        } else {
            throw std::invalid_argument(
                "kpsr::SafeEventEmitter::removeListener: Invalid listener id.");
        }
    }

    /**
     * @brief emitEvent
     * @param subscriberName
     * @param enqueuedTimeNs
     * @param args
     */
    void emitEvent(const std::string &subscriberName,
                   long long unsigned int enqueuedTimeNs,
                   const Event &event) override
    {
        std::vector<std::shared_ptr<ListenerBase>> handlers(listeners.size());
        {
            std::lock_guard<std::mutex> lock(mutex);

            auto range = listeners.equal_range(subscriberName);
            handlers.resize(std::distance(range.first, range.second));
            std::transform(range.first,
                           range.second,
                           handlers.begin(),
                           [](const std::pair<const std::string, std::shared_ptr<ListenerBase>> &p) {
                               return p.second;
                           });
        }

        for (auto &h : handlers) {
            h->runCallback(event, enqueuedTimeNs);
            if (h->isOnce) {
                removeListener(nullptr, h->id);
            }
        }
    }

    /**
     * @brief discardEvent
     * @param subscriberName Subscriber on which event is received.
     *
     */
    void discardEvent(const std::string &subscriberName) override
    {
        std::vector<std::shared_ptr<ListenerBase>> handlers(listeners.size());
        {
            std::lock_guard<std::mutex> lock(mutex);
            auto range = listeners.equal_range(subscriberName);
            handlers.resize(std::distance(range.first, range.second));
            std::transform(range.first,
                           range.second,
                           handlers.begin(),
                           [](const std::pair<const std::string, std::shared_ptr<ListenerBase>> &p) {
                               return p.second;
                           });
        }
        for (auto &listener : handlers) {
            auto listenerStatistics = listener->listenerStats;
            if (listenerStatistics) {
                listenerStatistics->totalDiscardedEvents++;
            } else {
                // Listener is not repeating. Statistics not present for listener.
                // Do nothing
            }
        }
    }

    /**
     * @brief getSubscriptionStats
     * @param listenerId
     * @return
     */
    std::shared_ptr<SubscriptionStats> getListenerStats(const unsigned int &listenerId) override
    {
        auto i = std::find_if(listeners.begin(),
                              listeners.end(),
                              [&](std::pair<const std::string, std::shared_ptr<ListenerBase>> p) {
                                  return p.second->id == listenerId;
                              });
        if (i == listeners.end()) {
            return nullptr;
        }
        return i->second->listenerStats;
    }

    void removeAllListeners(Container *container) override
    {
        if (container) {
            for (auto &subscriberListenerBasePair : listeners) {
                container->detach(subscriberListenerBasePair.second->listenerStats.get());
            }
        }
        // clear the map
        listeners.clear();
    }

private:
    struct ListenerBase
    {
        ListenerBase() {}

        ListenerBase(unsigned int id,
                     bool isOnce,
                     const std::function<void(const Event &event)> &callback,
                     std::shared_ptr<SubscriptionStats> listenerStats = nullptr)
            : id(id)
            , isOnce(isOnce)
            , callback(callback)
            , listenerStats(listenerStats)
        {}

        virtual ~ListenerBase() {}

        virtual void runCallback(const Event &event, long long unsigned int enqueuedTimeNs = 0)
        {
            callback(event);
        }

        unsigned int id;
        bool isOnce;
        std::function<void(const Event &event)> callback;
        std::shared_ptr<SubscriptionStats> listenerStats;
    };

    struct ListenerWithStats : public ListenerBase
    {
        using ListenerBase::ListenerBase; // reuse constructor from base class

        void runCallback(const Event &event, long long unsigned int enqueuedTimeNs) override
        {
            this->listenerStats->startProcessMeasure();
            if (enqueuedTimeNs > 0) {
                this->listenerStats->totalEnqueuedTimeInNs +=
                    (TimeUtils::getCurrentNanosecondsAsLlu() - enqueuedTimeNs);
            }
            this->callback(event);
            this->listenerStats->stopProcessMeasure();
        }
    };

    unsigned int addListener(Container *container,
                             const std::string &listenerName,
                             const std::string &subscriberName,
                             bool isOnce,
                             std::function<void(const Event &event)> callback)
    {
        std::lock_guard<std::mutex> lock(mutex);

        unsigned int listenerId = ++last_listener;
        std::shared_ptr<kpsr::SubscriptionStats> listenerStats(nullptr);
        if (!isOnce) {
            listenerStats = std::make_shared<kpsr::SubscriptionStats>(subscriberName,
                                                                      listenerName,
                                                                      EVENT_EMITTER_NAME);
            if (container != nullptr) {
                container->attach(listenerStats.get());
            }
            listeners.insert(std::make_pair(subscriberName,
                                            std::make_shared<ListenerWithStats>(listenerId,
                                                                                isOnce,
                                                                                callback,
                                                                                listenerStats)));
        } else {
            listeners.insert(
                std::make_pair(subscriberName,
                               std::make_shared<ListenerBase>(listenerId, isOnce, callback)));
        }
        return listenerId;
    }

    std::mutex mutex;
    unsigned int last_listener = 0;
    std::multimap<std::string, std::shared_ptr<ListenerBase>> listeners;

    SafeEventEmitter(const SafeEventEmitter &) = delete;
    const SafeEventEmitter &operator=(const SafeEventEmitter &) = delete;

    const std::string SINGLE_EXECUTING_LISTENER{"once"};
    const std::string EVENT_EMITTER_NAME{"EVENT_EMITTER"};
};
} // namespace kpsr

#endif
