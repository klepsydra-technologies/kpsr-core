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

#ifndef ZMQ_ENVIRONMENT_DATA_H
#define ZMQ_ENVIRONMENT_DATA_H

#include <string>

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>

namespace kpsr {
namespace zmq_mdlw {
/**
 * @brief The ZMQEnvironmentData struct
 *
 * @copyright 2023 Klepsydra Technologies AG
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
struct ZMQEnvironmentData
{
public:
    /**
     * @brief _configurationKey
     */
    std::string _configurationKey;

    /**
     * @brief _configurationData
     */
    std::string _configurationData;

    /**
     * @brief _sourceId
     */
    long _sourceId;

    /**
     * @brief ZMQEnvironmentData
     */
    ZMQEnvironmentData() {}

    /**
     * @brief ZMQEnvironmentData
     * @param configurationKey
     * @param configurationData
     * @param sourceId
     */
    ZMQEnvironmentData(const std::string &configurationKey,
                       const std::string &configurationData,
                       long sourceId)
        : _configurationKey(configurationKey)
        , _configurationData(configurationData)
        , _sourceId(sourceId)
    {}
};
} // namespace zmq_mdlw
} // namespace kpsr

namespace cereal {
template<class Archive>
/**
 * @brief serialize
 * @param archive
 * @param event
 */
void serialize(Archive &archive, kpsr::zmq_mdlw::ZMQEnvironmentData &event)
{
    archive(CEREAL_NVP(event._configurationKey),
            CEREAL_NVP(event._configurationData),
            CEREAL_NVP(event._sourceId));
}
} // namespace cereal

#endif // ZMQ_ENVIRONMENT_DATA_H
