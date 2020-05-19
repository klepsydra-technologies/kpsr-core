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

#ifndef BASIC_PUBLISHER_H
#define BASIC_PUBLISHER_H

#include <klepsydra/core/object_pool_publisher.h>

#include <klepsydra/mem_core/safe_queue.h>
#include <klepsydra/mem_core/basic_event_data.h>

namespace kpsr
{
namespace mem
{
template <class T>
/*!
 * \brief The BasicPublisher class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details Publishing class that puts events in the safequeue. It has additional configuration to optimise
 * memory allocation.
 *
 */
class BasicPublisher : public ObjectPoolPublisher<T>
{
public:
    /**
     * @brief BasicPublisher
     * @param container
     * @param name
     * @param poolSize
     * @param initializerFunction function to be invoked after event instantiaion.
     * @param eventCloner a function used to clone events after copied in the publish method.
     * @param safeQueue
     * @param discardItemsWhenFull when true, old events will be deleted when the queue is full and new one need to be put.
     * In the false case, the publisher will block until there is free space to put new events in the queue, if the queue
     * is full.
     */
    BasicPublisher(Container * container,
                   const std::string & name,
                   int poolSize,
                   std::function<void(T &)> initializerFunction,
                   std::function<void(const T &, T &)> eventCloner,
                   SafeQueue <EventData<const T>> & safeQueue,
                   bool discardItemsWhenFull)
        : ObjectPoolPublisher<T>(container, name, "SAFE_QUEUE", poolSize, initializerFunction, eventCloner)
        , _internalQueue(safeQueue)
        , _discardItemsWhenFull(discardItemsWhenFull)
    {}

    /**
     * @brief internalPublish publish by a safe queue push into queue.
     * @param eventData
     */
    void internalPublish(std::shared_ptr<const T> eventData) override {
        EventData<const T> safeQueueEvent;
        safeQueueEvent.enqueuedTimeInNs = TimeUtils::getCurrentNanosecondsAsLlu();
        safeQueueEvent.eventData = eventData;
        if (_discardItemsWhenFull) {
            // Non-blocking call
            uint discardedItems = _internalQueue.force_move_push(safeQueueEvent);
            this->_publicationStats._totalDiscardedEvents += discardedItems;
        } else {
            // Blocking call
            _internalQueue.move_push(safeQueueEvent);
        }
    }

private:
    SafeQueue <EventData<const T>> & _internalQueue;
    bool _discardItemsWhenFull;
};
}
}
#endif
