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

#ifndef EVENTLOOP_DATA_WRAPPER_H
#define EVENTLOOP_DATA_WRAPPER_H

#include <functional>
#include <memory>
#include <string>

namespace kpsr {
namespace high_performance {
/**
 * @brief The EventloopDataWrapper struct
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version 2.0.1
 *
 * @ingroup kpsr-eventloop-internal
 *
 */
struct EventloopDataWrapper
{
    std::string eventName;
    std::shared_ptr<const void> eventData;
    long long unsigned int enqueuedTimeInNs;
};
} // namespace high_performance
} // namespace kpsr

#endif // EVENTLOOP_DATA_WRAPPER_H
