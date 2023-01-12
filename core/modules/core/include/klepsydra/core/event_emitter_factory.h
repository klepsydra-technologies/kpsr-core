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
