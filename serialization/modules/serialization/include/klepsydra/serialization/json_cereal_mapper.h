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

#ifndef CEREAL_JSON_MAPPER_H
#define CEREAL_JSON_MAPPER_H

#include <sstream>

#include <cereal/archives/json.hpp>
#include <cereal/cereal.hpp>

#include <klepsydra/serialization/mapper.h>

namespace kpsr {
template<class T>
/**
 * @brief The Mapper<T, std::string> class
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-serialization-cereal
 *
 */
class Mapper<T, std::string>
{
public:
    /**
     * @brief fromMiddleware
     * @param message
     * @param event
     */
    void fromMiddleware(const std::string &message, T &event)
    {
        std::stringstream ss;
        ss.str(message);
        cereal::JSONInputArchive archive(ss);
        archive(CEREAL_NVP(event));
    }

    /**
     * @brief toMiddleware
     * @param event
     * @param message
     */
    void toMiddleware(const T &event, std::string &message)
    {
        std::stringstream ss;
        {
            cereal::JSONOutputArchive archive(ss);
            archive(CEREAL_NVP(event));
        }
        message = ss.str();
    }
};
} // namespace kpsr

#endif // CEREAL_JSON_MAPPER_H
