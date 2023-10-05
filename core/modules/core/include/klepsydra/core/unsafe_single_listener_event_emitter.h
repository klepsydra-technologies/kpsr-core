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

#ifndef SINGLE_LISTENER_EVENT_EMITTER_H
#define SINGLE_LISTENER_EVENT_EMITTER_H

#include <spdlog/spdlog.h>

#include <klepsydra/core/event_emitter_interface.h>
#include <klepsydra/sdk/time_utils.h>

namespace kpsr {
/**
 * @brief The SafeEventEmitter class
 *
 * @copyright 2023 Klepsydra Technologies AG
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
    UnsafeSingleListenerEventEmitter()
        : _listenerRegistered(false)
    {}

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
        spdlog::trace("{} listenerName: {}, subscriberName: {}.",
                      __PRETTY_FUNCTION__,
                      listenerName,
                      subscriberName);
        if (!_listenerRegistered) {
            _singleListener = callback;
            _listenerName = listenerName;
            _subscriberName = subscriberName;
            if (container) {
                _singleListenerStats = std::make_shared<SubscriptionStats>(subscriberName,
                                                                           listenerName,
                                                                           EVENT_EMITTER_NAME,
                                                                           true);
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
            _listenerRegistered = true;
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
            _listenerRegistered = false;
            _singleListener = [](const Event &event) {};
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
        spdlog::trace("{} subscriberName: {}.", __PRETTY_FUNCTION__, subscriberName);
        if ((subscriberName != _subscriberName) || !_listenerRegistered) {
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

    void removeAllListeners(Container *container) override { removeListener(container, 0); }

private:
    std::atomic<bool> _listenerRegistered;
    std::string _listenerName;
    std::string _subscriberName;
    std::shared_ptr<SubscriptionStats> _singleListenerStats = nullptr;
    std::function<void(const Event &event)> _singleListener = nullptr;

    std::function<void(const Event &event, long long unsigned int enqueuedTimeNs)>
        _singleListenerWrapper = nullptr;

    UnsafeSingleListenerEventEmitter(const UnsafeSingleListenerEventEmitter &) = delete;
    const UnsafeSingleListenerEventEmitter &operator=(const UnsafeSingleListenerEventEmitter &) =
        delete;
    const std::string EVENT_EMITTER_NAME{"UNSAFE_SINGLE_EVENT_EMITTER"};
};
} // namespace kpsr

#endif // SINGLE_LISTENER_EVENT_EMITTER_H
