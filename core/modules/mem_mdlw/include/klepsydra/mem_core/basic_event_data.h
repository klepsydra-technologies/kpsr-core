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

#ifndef BASIC_EVENT_DATA_H
#define BASIC_EVENT_DATA_H

#include <memory>

namespace kpsr {
namespace mem {
template<class T>
/**
 * @brief The EventData struct
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-mem-composition
 *
 * @details wrapper struct to be used when storing events in the queue.
 */
struct EventData
{
    /**
     * @brief eventData actual event
     */
    std::shared_ptr<T> eventData = nullptr;
    /**
     * @brief enqueuedTimeInNs timestamp at which the event was placed in the queue.
     */
    long long unsigned int enqueuedTimeInNs = 0;
};
} // namespace mem
} // namespace kpsr

#endif // BASIC_EVENT_DATA_H
