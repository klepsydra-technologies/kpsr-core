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

#ifndef FIXED_LISTENERS_EVENT_EMITTER_H
#define FIXED_LISTENERS_EVENT_EMITTER_H

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
class UnsafeMultiListenerEventEmitter : public EventEmitterInterface<Event>
{
public:
    /**
     * @brief SafeEventEmitter
     */
    UnsafeMultiListenerEventEmitter() {}

    ~UnsafeMultiListenerEventEmitter() {}

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
        spdlog::debug("{} listenerName: {}, subscriberName: {}.",
                      __PRETTY_FUNCTION__,
                      listenerName,
                      subscriberName);
        if (_listenersNameMap.find(listenerName) != _listenersNameMap.end()) {
            spdlog::info("Listener {} was already added.", listenerName);
            return _listenersNameMap[listenerName];
        } else {
            if (_subscriberNameMap.find(subscriberName) == _subscriberNameMap.end()) {
                _subscriberNameMap[subscriberName] = std::vector<unsigned int>(0);
            }
            unsigned int listenerId = _listeners.size();
            std::string listenerKeyName = listenerName;
            _listenersNameMap[listenerKeyName] = listenerId;
            if (container) {
                auto listenerStat = std::make_shared<SubscriptionStats>(subscriberName,
                                                                        listenerName,
                                                                        "EVENT_EMITTER");
                _listenerStats[listenerId] = listenerStat;
                container->attach(listenerStat.get());
                auto wrappedCallback = [listenerStat,
                                        callback](const Event &event,
                                                  long long unsigned int enqueuedTimeNs) {
                    listenerStat->startProcessMeasure();
                    if (enqueuedTimeNs > 0) {
                        listenerStat->totalEnqueuedTimeInNs +=
                            (TimeUtils::getCurrentNanosecondsAsLlu() - enqueuedTimeNs);
                    }
                    callback(event);
                    listenerStat->stopProcessMeasure();
                };
                _listeners.push_back(wrappedCallback);

            } else {
                auto wrappedCallback = [callback](const Event &event,
                                                  long long unsigned int enqueuedTimeNs) {
                    callback(event);
                };
                _listeners.push_back(wrappedCallback);
            }
            _subscriberNameMap[subscriberName].push_back(listenerId);
            return listenerId;
        }
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
        throw std::invalid_argument(
            "kpsr::UnsafeMultiListenerEventEmitter::once: Unsupported operation.");
    }

    /**
     * @brief removeListener
     * @param listenerId
     */
    void removeListener(Container *container, unsigned int listenerId) override
    {
        spdlog::debug("{} listenerId: {}", __PRETTY_FUNCTION__, listenerId);
        if (_listeners.size() > listenerId) {
            _listeners[listenerId] = nullptr;
            std::string listenerName;
            bool found = false;
            for (auto &it : _listenersNameMap) {
                if (it.second == listenerId) {
                    listenerName = it.first;
                    found = true;
                    break;
                }
            }
            if (found) {
                _listenersNameMap.erase(listenerName);
                if (container && (_listenerStats.find(listenerId) != _listenerStats.end())) {
                    container->detach(_listenerStats[listenerId].get());
                    _listenerStats.erase(listenerId);
                }
            }

            for (auto &subscriber : _subscriberNameMap) {
                auto &listenerIdVector = subscriber.second;
                auto elementToBeRemoved = std::find_if(listenerIdVector.begin(),
                                                       listenerIdVector.end(),
                                                       [&listenerId](
                                                           const unsigned int currentListenerId) {
                                                           return currentListenerId == listenerId;
                                                       });
                if (elementToBeRemoved != listenerIdVector.end()) {
                    listenerIdVector.erase(elementToBeRemoved);
                    if (listenerIdVector.empty()) {
                        _subscriberNameMap.erase(subscriber.first);
                    }
                    break;
                }
            }
        } else {
            spdlog::info("Listener {} cannot be removed because it is not registered.", listenerId);
        }
    }

    void removeAllListeners(Container *container) override
    {
        if (container) {
            for (auto listenerIdStatsPair : _listenerStats) {
                container->detach(listenerIdStatsPair.second.get());
            }
        }
        _listenersNameMap.clear();
        _subscriberNameMap.clear();
        _listenersNameMap.clear();
        _listeners.clear();
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
        for (auto &it : _subscriberNameMap[subscriberName]) {
            _listeners[it](event, enqueuedTimeNs);
        }
    }

    /**
     * @brief discardEvent
     * @param subscriberName Subscriber on which event is received.
     *
     */
    void discardEvent(const std::string &subscriberName) override
    {
        auto subscriberIterator = _subscriberNameMap.find(subscriberName);
        if (_subscriberNameMap.end() == subscriberIterator) {
            return;
        }
        auto &listenerIdList = subscriberIterator->second;
        for (auto &listenerId : listenerIdList) {
            auto listenerStatsIterator = _listenerStats.find(listenerId);
            if (_listenerStats.end() != listenerStatsIterator) {
                auto &listenerStats = listenerStatsIterator->second;
                listenerStats->totalDiscardedEvents++;
            } else {
                // Do nothing because listener does not have statistics
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
        if (_listenerStats.size() <= listenerId) {
            return nullptr;
        }
        return _listenerStats.at(listenerId);
    }

private:
    std::map<const std::string, unsigned int> _listenersNameMap;
    std::map<const std::string, std::vector<unsigned int>> _subscriberNameMap;
    std::map<unsigned int, std::shared_ptr<SubscriptionStats>> _listenerStats;
    std::vector<std::function<void(const Event &event, long long unsigned int enqueuedTimeNs)>>
        _listeners;

    UnsafeMultiListenerEventEmitter(const UnsafeMultiListenerEventEmitter &) = delete;
    const UnsafeMultiListenerEventEmitter &operator=(const UnsafeMultiListenerEventEmitter &) =
        delete;
};
} // namespace kpsr

#endif // FIXED_LISTENERS_EVENT_EMITTER_H
