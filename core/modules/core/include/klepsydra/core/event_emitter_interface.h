#ifndef EVENT_EMITTER_INTERFACE_H
#define EVENT_EMITTER_INTERFACE_H

#include <functional>
#include <memory>

#include <klepsydra/core/container.h>
#include <klepsydra/core/subscription_stats.h>

namespace kpsr {
/**
 * @brief The EventEmitterInterface class
 */
template<typename Event>
class EventEmitterInterface
{
public:
    /**
     * @brief on
     * @param eventId
     * @param listenerName
     * @param callback
     * @return
     */
    virtual unsigned int on(Container *container,
                            const std::string &eventId,
                            const std::string &listenerName,
                            std::function<void(const Event &event)> callback) = 0;

    /**
     * @brief once
     * @param eventId
     * @param callback
     * @return
     */
    virtual unsigned int once(const std::string &eventId,
                              std::function<void(const Event &event)> callback) = 0;

    /**
     * @brief removeListener
     * @param listenerId
     */
    virtual void removeListener(Container *container, unsigned int listenerId) = 0;

    /**
     * @brief emitEvent
     * @param eventId
     * @param enqueuedTimeNs
     * @param args
     */
    virtual void emitEvent(const std::string &eventId,
                           long long unsigned int enqueuedTimeNs,
                           const Event &args) = 0;

    /**
     * @brief discardEvent
     * @param eventId Subscriber on which event is received.
     *
     */
    virtual void discardEvent(const std::string &eventId) = 0;

    /**
     * @brief getSubscriptionStats
     * @param listenerId
     * @return
     */
    virtual std::shared_ptr<SubscriptionStats> getListenerStats(const unsigned int &listenerId) = 0;

    /**
     * @brief removeAllListeners
     *
     */
    virtual void removeAllListeners(Container *container) = 0;
};
} // namespace kpsr

#endif // EVENT_EMITTER_INTERFACE_H
