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

#ifndef IDENTITY_MAPPER_H
#define IDENTITY_MAPPER_H

#include <cstring>
#include <iostream>
#include <vector>

#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class T>
/**
 * @brief The Mapper<T, std::vector<unsigned char> > class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization
 *
 */
class Mapper<T, std::vector<unsigned char>>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std::vector<unsigned char> &message, T &event)
    {
        std::memcpy(&event, message.data(), sizeof(T));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T &event, std::vector<unsigned char> &message)
    {
        std::vector<unsigned char> data((unsigned char *) (&event),
                                        (unsigned char *) &event + sizeof(T));
        message = data;
    }
};
} // namespace kpsr

#endif // IDENTITY_MAPPER_H
