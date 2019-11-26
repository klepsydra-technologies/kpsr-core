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

#ifndef EVENT_TRANSFORM_FORWARDER_H
#define EVENT_TRANSFORM_FORWARDER_H

#include <functional>
#include <atomic>

#include <klepsydra/core/publisher.h>

namespace kpsr
{
template <class T, class U>
/*!
 * @brief The EventTransformForwarder class
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-application
 *
 * @details An optimised forwarding helper class. Its purpose is to optimise memory allocation and callstack when processing events,
 * transforming and forwarding them.
 */
class EventTransformForwarder {
public:
    /*!
     * @brief EventTransformForwarder
     * @param transformFunction function that process and event and transforms it into another object.
     * @param destPublisher forwarding publisher.
     */
    EventTransformForwarder(std::function<void(const T &, U &)> transformFunction,
                            Publisher<U> * destPublisher)
        : forwarderListenerFunction(std::bind(&kpsr::EventTransformForwarder<T, U>::onEventReceived, this, std::placeholders::_1))
        , _transformFunction(transformFunction)
        , _destPublisher(destPublisher)
        , _eventReference(nullptr)
    {
        _process = [this] (U & newEvent) {
            this->_transformFunction(* _eventReference, newEvent);
        };
    }

    /*!
     * @brief forwarderListenerFunction std::function to be added to the source subcriber.
     */
    std::function<void(const T &)> forwarderListenerFunction;

    /*!
     * \brief onEventReceived
     * \param event
     */
    void onEventReceived(const T & event) {
        _eventReference = &event;
        _destPublisher->processAndPublish(_process);
    }

private:
    std::function<void(const T &, U &)> _transformFunction;
    Publisher<U> * _destPublisher;
    std::atomic<const T *> _eventReference;
    std::function<void(U &)> _process;
};
}
#endif // EVENT_TRANSFORM_FORWARDER_H
