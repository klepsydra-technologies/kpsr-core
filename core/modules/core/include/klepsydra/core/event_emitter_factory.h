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

#ifndef EVENT_EMITTER_FACTORY_H
#define EVENT_EMITTER_FACTORY_H

#include <memory>

#include <klepsydra/core/safe_event_emitter.h>
#include <klepsydra/core/unsafe_multi_listener_event_emitter.h>
#include <klepsydra/core/unsafe_single_listener_event_emitter.h>

namespace kpsr {

enum EventEmitterType { UNSAFE_SINGLE, UNSAFE_MULTI, SAFE };

class EventEmitterFactory
{
public:
    template<class T>
    static std::shared_ptr<EventEmitterInterface<T>> createEventEmitter(
        EventEmitterType eventEmitterType)
    {
        std::shared_ptr<EventEmitterInterface<T>> eventEmitter;
        switch (eventEmitterType) {
        case UNSAFE_SINGLE:
            eventEmitter = std::make_shared<UnsafeSingleListenerEventEmitter<T>>();
            break;
        case UNSAFE_MULTI:
            eventEmitter = std::make_shared<UnsafeMultiListenerEventEmitter<T>>();
            break;
        case SAFE:
            eventEmitter = std::make_shared<SafeEventEmitter<T>>();
            break;
        }
        return eventEmitter;
    }
};
} // namespace kpsr

#endif // EVENT_EMITTER_FACTORY_H
