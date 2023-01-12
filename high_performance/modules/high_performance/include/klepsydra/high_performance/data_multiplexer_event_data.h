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

#ifndef DATA_MULTIPLEXER_EVENT_DATA_H
#define DATA_MULTIPLEXER_EVENT_DATA_H

#include <memory>

namespace kpsr {
namespace high_performance {
template<class T>
/**
 * @brief The EventData struct
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-high_performance-internal
 *
 * @details Internal wrapper class that is actually stored in the high_performance and eventloop.
 */
struct DataMultiplexerDataWrapper
{
    DataMultiplexerDataWrapper()
        : eventData(std::make_shared<T>())
    {}

    DataMultiplexerDataWrapper(std::shared_ptr<T> eventData)
        : eventData(eventData)
    {}

    /**
     * @brief eventData
     */
    std::shared_ptr<T> eventData;

    /**
     * @brief enqueuedTimeInNs
     */
    long long unsigned int enqueuedTimeInNs;
};
} // namespace high_performance
} // namespace kpsr

#endif // DATA_MULTIPLEXER_EVENT_DATA_H
