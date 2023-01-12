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

#ifndef CEREAL_BINARY_MAPPER_H
#define CEREAL_BINARY_MAPPER_H

#include <memory>
#include <streambuf>

#include <cereal/archives/portable_binary.hpp>
#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>

#include <klepsydra/serialization/mapper.h>

using Base = std::basic_streambuf<char> *;

namespace kpsr {
template<class T>
/**
 * @brief The Mapper<T, Base> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization-cereal
 *
 */
class Mapper<T, Base>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const Base &message, T &event)
    {
        std::istream iss(message);
        cereal::PortableBinaryInputArchive archive(iss);
        archive(CEREAL_NVP(event));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T &event, Base &message)
    {
        std::ostream oss(message);
        {
            cereal::PortableBinaryOutputArchive archive(oss);
            archive(CEREAL_NVP(event));
        }
    }
};
} // namespace kpsr

#endif // CEREAL_BINARY_MAPPER_H
