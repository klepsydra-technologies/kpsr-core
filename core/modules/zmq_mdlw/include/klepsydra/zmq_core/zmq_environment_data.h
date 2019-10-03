/****************************************************************************
*
*                           Klepsydra Core Modules
*              Copyright (C) 2019-2020  Klepsydra Technologies GmbH
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*
****************************************************************************/

#ifndef ZMQ_ENVIRONMENT_DATA_H
#define ZMQ_ENVIRONMENT_DATA_H

#include <string>

#include <cereal/cereal.hpp>
#include <cereal/types/string.hpp>

namespace kpsr
{
namespace zmq_mdlw
{
/**
 * @brief The ZMQEnvironmentData struct
 *
 * @copyright Klepsydra Technologies 2019-2020.
 *
 * @version   2.1.0
 *
 * @ingroup kpsr-zmq-internal
 *
 */
struct ZMQEnvironmentData {
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
    ZMQEnvironmentData(std::string configurationKey, std::string configurationData, long sourceId)
        : _configurationKey(configurationKey)
        , _configurationData(configurationData)
        , _sourceId(sourceId)
    {}
};
}
}

namespace cereal {
template<class Archive>
/**
 * @brief serialize
 * @param archive
 * @param event
 */
void serialize(Archive & archive, kpsr::zmq_mdlw::ZMQEnvironmentData & event)
{
    archive( CEREAL_NVP(event._configurationKey),
             CEREAL_NVP(event._configurationData),
             CEREAL_NVP(event._sourceId) );
}
}

#endif // ZMQ_ENVIRONMENT_DATA_H
