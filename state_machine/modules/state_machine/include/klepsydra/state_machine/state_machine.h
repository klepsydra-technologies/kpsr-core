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

#pragma once

#include <functional>
#include <memory>
#include <string>

namespace kpsr {
namespace fsm {
class StateMachine
{
public:
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void update() = 0;
    virtual void enqueueAndUpdate(const std::string &event) = 0;
    virtual void reset() = 0;
    virtual void enqueueEvent(const std::string &event) = 0;
    virtual void registerObserver(
        std::function<void(const std::string &currentState, bool stateChanged)> smObserver) = 0;
    virtual void unregisterObservers() = 0;
};
} // namespace fsm
} // namespace kpsr
