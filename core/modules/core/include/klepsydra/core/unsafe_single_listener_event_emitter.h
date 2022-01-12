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

#ifndef SINGLE_LISTENER_EVENT_EMITTER_H
#define SINGLE_LISTENER_EVENT_EMITTER_H

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
class UnsafeSingleListenerEventEmitter : public EventEmitterInterface<Event>
{
public:
    /**
     * @brief SafeEventEmitter
     */
    UnsafeSingleListenerEventEmitter() {}

    ~UnsafeSingleListenerEventEmitter() {}

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
        if (_singleListener == nullptr) {
            _singleListener = callback;
            _listenerName = listenerName;
            _subscriberName = subscriberName;
            if (container != nullptr) {
                _singleListenerStats = std::make_shared<SubscriptionStats>(subscriberName,
                                                                           listenerName,
                                                                           EVENT_EMITTER_NAME);
                container->attach(_singleListenerStats.get());

                _singleListenerWrapper = [this](const Event &event,
                                                long long unsigned int enqueuedTimeNs) {
                    this->_singleListenerStats->startProcessMeasure();
                    if (enqueuedTimeNs > 0) {
                        this->_singleListenerStats->totalEnqueuedTimeInNs +=
                            (TimeUtils::getCurrentNanosecondsAsLlu() - enqueuedTimeNs);
                    }
                    this->_singleListener(event);
                    this->_singleListenerStats->stopProcessMeasure();
                };
            } else {
                _singleListenerWrapper = [this](const Event &event,
                                                long long unsigned int enqueuedTimeNs) {
                    this->_singleListener(event);
                };
            }
        }
        return 0;
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
            "kpsr::UnsafeSingleListenerEventEmitter::once: Unsupported operation.");
    }

    /**
     * @brief removeListener
     * @param listenerId
     */
    void removeListener(Container *container, unsigned int listenerId) override
    {
        if (listenerId == 0) {
            _singleListener = nullptr;
            if (container) {
                container->detach(_singleListenerStats.get());
                _singleListenerStats.reset();
            }
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
        if ((subscriberName != _subscriberName) || !_singleListener) {
            return;
        }
        _singleListenerWrapper(event, enqueuedTimeNs);
    }

    /**
     * @brief discardEvent
     * @param subscriberName Subscriber on which event is received.
     *
     */
    void discardEvent(const std::string &subscriberName) override
    {
        if (subscriberName != _subscriberName) {
            return;
        }
        if (_singleListenerStats) {
            _singleListenerStats->totalDiscardedEvents++;
        }
    }

    /**
     * @brief getSubscriptionStats
     * @param listenerId
     * @return
     */
    std::shared_ptr<SubscriptionStats> getListenerStats(const unsigned int &listenerId) override
    {
        if (listenerId == 0) {
            return _singleListenerStats;
        }
        return nullptr;
    }

    void removeAllListeners(Container *container) override
    {
        if (container) {
            container->detach(_singleListenerStats.get());
        }
        _singleListenerStats.reset();
        _singleListener = nullptr;
    }

private:
    std::string _listenerName;
    std::string _subscriberName;
    std::shared_ptr<SubscriptionStats> _singleListenerStats = nullptr;
    std::function<void(const Event &event)> _singleListener = nullptr;

    std::function<void(const Event &event, long long unsigned int enqueuedTimeNs)>
        _singleListenerWrapper = nullptr;

    UnsafeSingleListenerEventEmitter(const UnsafeSingleListenerEventEmitter &) = delete;
    const UnsafeSingleListenerEventEmitter &operator=(const UnsafeSingleListenerEventEmitter &) =
        delete;
    const std::string EVENT_EMITTER_NAME{"EVENT_EMITTER"};
};
} // namespace kpsr

#endif // SINGLE_LISTENER_EVENT_EMITTER_H
