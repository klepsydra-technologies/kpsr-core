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

#ifndef EVENT_EMITTER_PUBLISHER_H
#define EVENT_EMITTER_PUBLISHER_H

#include <klepsydra/core/object_pool_publisher.h>
#include <klepsydra/core/event_emitter.h>

/**
*/
namespace kpsr
{
template<class T>
/*!
 * @brief The EventEmitterPublisher class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-test
 *
 * @details This publisher implementation is intended for testing purposes. It is synchronous and single-threaded. The goal is to use this publisher as a testing publisher for unit and integration testing. It derives from the ObjectPoolPublisher, which
 * allows to pre-allocate the message instances in a pool for increasing the performance.
 *
 */
class EventEmitterPublisher : public ObjectPoolPublisher<T>
{
public:
    /*!
     * @brief EventEmitterPublisher
     * @param container
     * @param eventName
     * @param eventEmitter
     * @param poolSize
     * @param initializerFunction
     * @param eventCloner
     */
    EventEmitterPublisher(Container * container,
                          std::string eventName,
                          EventEmitter & eventEmitter,
                          int poolSize,
                          std::function<void(T &)> initializerFunction,
                          std::function<void(const T &, T &)> eventCloner)
        : ObjectPoolPublisher<T>(container, eventName, "EVENT_EMITTER", poolSize, initializerFunction, eventCloner)
        , _eventEmitter(eventEmitter)
        , _eventName(eventName)
    {}

    /*!
     * @brief internalPublish publish events without copying.
     * @param event
     */
    void internalPublish(std::shared_ptr<const T> event) override {
        _eventEmitter.emitEvent(_eventName, 0, *event.get());
    }

private:
    EventEmitter & _eventEmitter;
    std::string _eventName;
};
}
#endif
